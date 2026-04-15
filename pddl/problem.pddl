(define (problem bookstore_problem)
(:domain bookstore_domain)

(:objects
  shelf_red shelf_green shelf_yellow shelf_blue
  middle_path deposit_table reception - location
  curiosity - robot
  red_book green_book yellow_book blue_book - item
)

(:init
  (is_book red_book)
  (is_book green_book)
  (is_book yellow_book)
  (is_book blue_book)

  (is_deposit deposit_table)
  (is_base reception)

  (robot_at curiosity reception)
  (gripper_free curiosity)
  (doing_nthg curiosity)

  (object_at red_book shelf_red)
  (object_at green_book shelf_green)
  (object_at yellow_book shelf_yellow)
  (object_at blue_book shelf_blue)
)

(:goal
  (and
    (book_deposited red_book)
    (book_deposited green_book)
    (book_deposited yellow_book)
    (book_deposited blue_book)
    (robot_at curiosity reception)
  )
)

)
