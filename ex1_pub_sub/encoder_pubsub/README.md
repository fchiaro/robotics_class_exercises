# Publisher-subscriber exercise

Implements a publisher that simulates a set of sensors for encoder readings (position of 6 joints) and a subscriber that simulates a set of controllers that prints such readings to stdout.

## Structure

Encoder readings are represented with `Reading` messages from the `encoder_pubsub_msgs` package.

## Usage

`rosrun encoder_pubsub publisher_node`

`rosrun encoder_pubsub subscriber_node`
