#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_solver/SolverClient.hpp"
#include "plansys2_solver/SolverNode.hpp"

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Reception : public rclcpp::Node
{
public:
  Reception()
  : rclcpp::Node("reception_controller"), state_(STARTING)
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    // Ensure solver_timeout is set before SolverClient reads it
    if (!has_parameter("solver_timeout")) {
      declare_parameter<double>("solver_timeout", 240.0);
    }
    set_parameter(rclcpp::Parameter("solver_timeout", 240.0));
    solver_client_ = std::make_shared<plansys2::SolverClient>();

    // Perception subscription — accumulates observations as backup context.
    // Move BT nodes also capture perception via out_msg for the action hub,
    // but timing issues mean the hub may not always contain perception data.
    // This log is included in the solver question as additional context.
    perception_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception_events", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        perception_log_.push_back(msg->data);
        RCLCPP_INFO(get_logger(), "[PERCEPTION] Logged: %s", msg->data.c_str());
      });

    init_knowledge();

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"shelf_red", "location"});
    problem_expert_->addInstance(plansys2::Instance{"shelf_green", "location"});
    problem_expert_->addInstance(plansys2::Instance{"shelf_yellow", "location"});
    problem_expert_->addInstance(plansys2::Instance{"shelf_blue", "location"});
    problem_expert_->addInstance(plansys2::Instance{"middle_path", "location"});
    problem_expert_->addInstance(plansys2::Instance{"deposit_table", "location"});
    problem_expert_->addInstance(plansys2::Instance{"reception", "location"});

    problem_expert_->addInstance(plansys2::Instance{"curiosity", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"red_book", "item"});
    problem_expert_->addInstance(plansys2::Instance{"green_book", "item"});
    problem_expert_->addInstance(plansys2::Instance{"yellow_book", "item"});
    problem_expert_->addInstance(plansys2::Instance{"blue_book", "item"});

    problem_expert_->addPredicate(plansys2::Predicate("(is_book red_book)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_book green_book)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_book yellow_book)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_book blue_book)"));

    problem_expert_->addPredicate(plansys2::Predicate("(is_deposit deposit_table)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_base reception)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at curiosity reception)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_free curiosity)"));
    problem_expert_->addPredicate(plansys2::Predicate("(doing_nthg curiosity)"));

    // All books start at their expected shelves in the PDDL symbolic state.
    // The planner believes every book is in its predefined place.
    problem_expert_->addPredicate(plansys2::Predicate("(object_at red_book shelf_red)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at green_book shelf_green)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at yellow_book shelf_yellow)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at blue_book shelf_blue)"));

    // Choose one random book to be displaced (not physically at its shelf).
    // The PDDL state still says it's there — the robot will discover it's missing
    // when it tries to pick it up, triggering the LLM-based replanning.
    choose_displaced_book();
  }

  void choose_displaced_book()
  {
    // The displaced book can be set via parameter (from launch file) or defaults
    // to random selection. This same parameter must be passed to the deposit_book
    // BT action node so CheckBookPresent knows which book to fail on.
    declare_parameter<std::string>("displaced_book", "");
    displaced_book_ = get_parameter("displaced_book").as_string();

    if (displaced_book_.empty()) {
      std::vector<std::string> books = {
        "red_book", "green_book", "yellow_book", "blue_book"};

      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dist(0, books.size() - 1);

      displaced_book_ = books[dist(gen)];
    }

    RCLCPP_WARN(get_logger(),
      "[SCENARIO] Displaced book: %s (not physically at its shelf, "
      "but PDDL still believes it is)", displaced_book_.c_str());
  }

  void step()
  {
    switch (state_)
    {
      case STARTING:
        RCLCPP_INFO(get_logger(), "State: STARTING -> PLANNING");
        state_ = PLANNING;
        break;

      case PLANNING:
        {
          RCLCPP_INFO(get_logger(), "State: PLANNING");

          problem_expert_->setGoal(plansys2::Goal(
            "(and (book_deposited red_book) (book_deposited green_book) "
            "(book_deposited yellow_book) (book_deposited blue_book) "
            "(robot_at curiosity reception))"));

          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            RCLCPP_ERROR(get_logger(), "Could not find plan to reach goal %s",
              parser::pddl::toString(problem_expert_->getGoal()).c_str());
            state_ = FINISH;
            break;
          }

          RCLCPP_INFO(get_logger(), "Plan found, starting execution");
          if (executor_client_->start_plan_execution(plan.value())) {
            state_ = WORKING;
          }
          break;
        }

      case WORKING:
        {
          // Perception events are accumulated passively in the background.
          // No cancellation here — let the plan run until it succeeds or fails.

          if (!executor_client_->execute_and_check_plan()) {
            auto result = executor_client_->getResult();

            if (result.value().result ==
                plansys2_msgs::action::ExecutePlan::Result::SUCCESS)
            {
              RCLCPP_INFO(get_logger(), "Plan successfully finished");
              state_ = FINISH;
            } else {
              // Plan failed — likely because a book was not found at its expected shelf.
              // Query the LLM solver with the accumulated perception log so it can
              // reason about where the missing book actually is.
              RCLCPP_ERROR(get_logger(),
                "[EXECUTION_FAILURE] Plan failed. Querying solver with perception log...");

              // Restore idle state after failure (at-start effects may have removed these).
              // pick_book removes doing_nthg and gripper_free at-start; if it failed
              // mid-way, we need to restore them for the replanned plan to work.
              problem_expert_->addPredicate(plansys2::Predicate("(doing_nthg curiosity)"));
              problem_expert_->addPredicate(plansys2::Predicate("(gripper_free curiosity)"));
              RCLCPP_INFO(get_logger(),
                "[RECOVERY] Forced idle-state recovery after execution failure "
                "(first-version assumption: doing_nthg + gripper_free restored)");

              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();

              // Include perception observations in the prompt so the LLM can
              // reason about where the missing book actually is.
              std::string perception_context = build_perception_context();

              std::string prompt =
                "EXECUTION FAILURE: A pick_book action failed because the book "
                "was not found at its expected location.\n\n"
                "The robot's perception system recorded these observations during "
                "navigation:\n" + perception_context + "\n"
                "Use the perception observations to determine where the missing "
                "book actually is.\n\n"
                "STRICT RULES:\n"
                "- Identify which book was not found at its expected shelf\n"
                "- Check the perception log for sightings of that book elsewhere\n"
                "- You may ONLY modify object_at predicates for the missing book\n"
                "- Do NOT modify robot_at, doing_nthg, gripper_free, robot_carrying, "
                "book_deposited, goals, or predicates for any other object\n"
                "- Reply ONLY with JSON, no extra text\n";

              auto solver_result = solver_client_->getReplanificateSolve(
                domain, problem, prompt, "");

              if (!solver_result.has_value()) {
                RCLCPP_ERROR(get_logger(), "[EXECUTION_FAILURE] Solver did not respond");
                state_ = FINISH;
              } else if (solver_result->classification ==
                plansys2_solver_msgs::msg::Solver::CORRECT)
              {
                RCLCPP_INFO(get_logger(),
                  "[EXECUTION_FAILURE] LLM: no state changes needed, replanning anyway");
                state_ = PLANNING;
              } else {
                RCLCPP_INFO(get_logger(),
                  "[EXECUTION_FAILURE] Applying LLM state corrections...");
                for (const auto & pred_str : solver_result->remove_predicates) {
                  bool ok = problem_expert_->removePredicate(plansys2::Predicate(pred_str));
                  RCLCPP_INFO(get_logger(), "REMOVE %s: %s",
                    pred_str.c_str(), ok ? "ok" : "failed");
                }
                for (const auto & pred_str : solver_result->add_predicates) {
                  bool ok = problem_expert_->addPredicate(plansys2::Predicate(pred_str));
                  RCLCPP_INFO(get_logger(), "ADD %s: %s",
                    pred_str.c_str(), ok ? "ok" : "failed");
                }
                state_ = PLANNING;
              }
            }
          }
          break;
        }

      case FINISH:
        {
          RCLCPP_INFO(get_logger(), "State: FINISH");
          rclcpp::shutdown();
          break;
        }

      default:
        break;
    }
  }

  std::string build_perception_context()
  {
    if (perception_log_.empty()) {
      return "(no perception events recorded)";
    }
    std::string context;
    for (size_t i = 0; i < perception_log_.size(); ++i) {
      context += "[" + std::to_string(i + 1) + "] " + perception_log_[i] + "\n";
    }
    return context;
  }



  // Getter so perception_sim or test code can know which book is displaced
  const std::string & get_displaced_book() const { return displaced_book_; }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::shared_ptr<plansys2::SolverClient> solver_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_sub_;
  std::vector<std::string> perception_log_;

  std::string displaced_book_;

  typedef enum {STARTING, PLANNING, WORKING, FINISH} StateType;
  StateType state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto reception_node = std::make_shared<Reception>();

  // Ensure the reception node has solver_timeout so SolverClient picks it up
  if (!reception_node->has_parameter("solver_timeout")) {
    reception_node->declare_parameter<double>("solver_timeout", 240.0);
  }

  auto solver_node = std::make_shared<plansys2::SolverNode>();

  // Set LLAMA solver as default if no params file was provided via --ros-args.
  {
    auto plugins = solver_node->get_parameter("solver_plugins").as_string_array();
    if (plugins.empty()) {
      solver_node->set_parameter(
        rclcpp::Parameter("solver_plugins", std::vector<std::string>{"LLAMA"}));
      solver_node->declare_parameter<std::string>(
        "LLAMA.plugin", "plansys2/LLAMASolver");
      solver_node->set_parameter(rclcpp::Parameter("solver_timeout", 240.0));
      RCLCPP_INFO(solver_node->get_logger(),
        "No solver params file provided, defaulting to LLAMA plugin");
    }
  }

  try {
    solver_node->configure();
  } catch (const std::exception & e) {
    std::cerr << "Error creating SolverNode: " << e.what() << std::endl;
    return 1;
  }

  if (!reception_node->init()) return 0;

  rclcpp::executors::SingleThreadedExecutor solver_executor;
  solver_executor.add_node(solver_node->get_node_base_interface());
  std::thread solver_thread([&solver_executor]() { solver_executor.spin(); });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(reception_node);

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    reception_node->step();
    executor.spin_some();
    rate.sleep();
  }

  solver_executor.cancel();
  solver_thread.join();

  rclcpp::shutdown();
  return 0;
}
