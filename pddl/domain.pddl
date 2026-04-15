(define (domain bookstore_domain)
(:requirements :strips :typing :durative-actions)
(:types
    location
    robot
    item
)

(:predicates
  (robot_at ?r - robot ?l - location)
  (object_at ?o - item ?l - location)
  (gripper_free ?r - robot)
  (robot_carrying ?r - robot ?b - item)
  (doing_nthg ?r - robot)
  (book_deposited ?o - item)
  (is_book ?b - item)
  (is_deposit ?l - location)
  (is_base ?l - location)
)

; Robot navigates between two locations.
; All effects at-end so cancellation leaves robot_at intact (first-version recovery model).
(:durative-action move
  :parameters (?r - robot ?from ?to - location)
  :duration (= ?duration 5)
  :condition
    (and
      (at start (robot_at ?r ?from))
      (at start (doing_nthg ?r))
    )
  :effect
    (and
      (at start (not (doing_nthg ?r)))
      (at end (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (doing_nthg ?r))
    )
)

; Pick a book at the robot's current location.
; Robot must be at the same location as the book and have a free gripper.
(:durative-action pick_book
  :parameters (?r - robot ?b - item ?l - location)
  :duration (= ?duration 2)
  :condition
    (and
      (over all (is_book ?b))
      (at start (robot_at ?r ?l))
      (at start (object_at ?b ?l))
      (at start (gripper_free ?r))
      (at start (doing_nthg ?r))
    )
  :effect
    (and
      (at start (not (doing_nthg ?r)))
      (at start (not (gripper_free ?r)))
      (at start (not (object_at ?b ?l)))
      (at end (robot_carrying ?r ?b))
      (at end (doing_nthg ?r))
    )
)

; Place a book the robot is carrying at a deposit location.
; The book becomes deposited.
(:durative-action place_book
  :parameters (?r - robot ?b - item ?l - location)
  :duration (= ?duration 2)
  :condition
    (and
      (over all (is_book ?b))
      (over all (is_deposit ?l))
      (at start (robot_at ?r ?l))
      (at start (robot_carrying ?r ?b))
      (at start (doing_nthg ?r))
    )
  :effect
    (and
      (at start (not (doing_nthg ?r)))
      (at start (not (robot_carrying ?r ?b)))
      (at end (gripper_free ?r))
      (at end (book_deposited ?b))
      (at end (doing_nthg ?r))
    )
)

)
