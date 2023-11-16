
# controllo nel camera_node quanto detection il marker
# e mando all action_server nel goal
if x_marker > self.x and y_marker > self.y:
    a = 1
    b = 1
elif x_marker < self.x and y_marker > self.y:
    a = -1
    b = 1
elif x_marker < self.x and y_marker < self.y:
    a = -1
    b = -1
elif x_marker > self.x and y_marker < self.y:
    a = 1
    b = -1



x_gol = 1.5
y_gol = 1.5
theta_gol = 1.57
cmd_vel = Twist()
if a*(x_gol-self.x)<0.1 or b*(y_gol-self.y)<0.1:
    if theta_gol - self.theta > 0.05:
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.5
    else:
        e_d = math.sqrt((self.x - 1.5)**2 + (self.y - 1.5)**2)
        cmd_vel.linear.x = float(e_d)
        cmd_vel.angular.z = 0.0
        if e_d <= 0.5:
            cmd_vel.linear.x = 0.0
else:
    e_a = theta_gol - self.theta
    e_d = math.sqrt((self.x - 1.5)**2 + (self.y - 1.5)**2)
    if e_d <= 0.1:
        e_d = 0.0
        e_a = 0.0
    # Compute control inputs
    lin_control = 0.3 * e_d
    ang_control = 0.3 * e_a
    # Build twist msg
    cmd_vel.linear.x = float(lin_control)
    cmd_vel.angular.z = float(ang_control)
self.publisher_.publish(cmd_vel)
self.get_logger().info('X: "%f"' % (self.x))
self.get_logger().info('Y: "%f"' % (self.y))
self.get_logger().info('Theta: "%f"' % (self.theta))

# oppure quando sono a una certa distanza faccio la traiettoria
# rischio che ottiene l'allineamento di theta gol prima dell arrivo al marker:
# non ruota più e va sempre dritto per sempre
# ma io devo arrivare AL marker guardandolo da li => devo forzarlo ad andare
# a self.x > x_marker se son nel primo quadrante col marker...