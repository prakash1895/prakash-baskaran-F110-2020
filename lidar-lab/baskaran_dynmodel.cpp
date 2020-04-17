

// Dynamical model of the car

double g = 9.81; // m/s^2

// compute first derivatives of state
double x_dot = start.velocity * std::cos(start.theta + start.slip_angle);
double y_dot = start.velocity * std::sin(start.theta + start.slip_angle);
double v_dot = accel;
double steer_angle_dot = steer_angle_vel;
double theta_dot = start.angular_velocity;

// for eases of next two calculations
double rear_val = g * p.l_r - accel * p.h_cg;
double front_val = g * p.l_f + accel * p.h_cg;

// in case velocity is 0
double vel_ratio, first_term;
if (start.velocity == 0) {
    vel_ratio = 0;
    first_term = 0;
}
else {
    vel_ratio = start.angular_velocity / start.velocity;
    first_term = p.friction_coeff / (start.velocity * (p.l_r + p.l_f));
}

double theta_double_dot = (p.friction_coeff * p.mass / (p.I_z * p.wheelbase)) *
        (p.l_f * p.cs_f * start.steer_angle * (rear_val) +
        start.slip_angle * (p.l_r * p.cs_r * (front_val) - p.l_f * p.cs_f * (rear_val)) -
        vel_ratio * (std::pow(p.l_f, 2) * p.cs_f * (rear_val) + std::pow(p.l_r, 2) * p.cs_r * (front_val)));\

double slip_angle_dot = (first_term) *
        (p.cs_f * start.steer_angle * (rear_val) -
        start.slip_angle * (p.cs_r * (front_val) + p.cs_f * (rear_val)) +
        vel_ratio * (p.cs_r * p.l_r * (front_val) - p.cs_f * p.l_f * (rear_val))) -
        start.angular_velocity;