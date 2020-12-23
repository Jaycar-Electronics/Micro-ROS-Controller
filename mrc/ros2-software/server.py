#!/usr/bin/python3
from .connector import MRCNode


def main(args=None):

    # Start up the ROS connector
    rclpy.init(args=args)

    #Start a new node, with the correct subscriber and publisher
    #note you need to specify the key word. this is by design.
    node = MyNode(
            publish_to_topic='/turtle1/cmd_vel',
            subscribe_to_topic='/turtle1/pose',
    )

    # Here we define the "view" function.
    def update_buttons():
        ''' get the button from form data, publish message in ros '''
        if request.method.lower() == 'get':
            return node.pose_status()

        button = request.form.get('button', None)

        if not button or button == "None":
            return node.pose_status()

        msg = Twist()

        if button == 'up':
            msg.linear.x = 2.0
        elif button == 'down':
            msg.linear.x = -2.0
        elif button == 'left':
            msg.angular.z = 2.0
        elif button == 'right':
            msg.angular.z = -2.0
        elif button == 'select':
            pass

        node.publisher.publish(msg)
        node.get_logger().info(
            f'posted data, received "{button}".'
        )  # Works fine

        return node.pose_status()

    node.app.add_url_rule(
        '/', 'button', view_func=update_buttons, methods=['POST', 'GET'])

    # SocketIO's run has eventlet sleeps incorporated when eventlet is imported
    node.socketio.run(node.app, host='0.0.0.0')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()