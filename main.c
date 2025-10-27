#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define M_PI 3.14159265358979323846 // seems like compiler does not have it

#define OUTPUT_MAX 20.0f // max value of signal in our case let it be 10 becuase values might be large
#define OUTPUT_MIN 0.0f  // min value of signal

/*
Simple PID controller

Output u(t) propostional to e(t) + e(t) + d/dt e(t)
u(t) = kp e(t) + Ki  e(t) + kd d/dt (et)
Using Laplace transform on both sides for deriving transfer function
u(s) = KpE(s) + Ki 1/s E(s) + Kd  s E(s)
u(s) = E(s) (Kp + Ki/s + Kd S)

By definition the transform function is output transfer / input transfer.
u(s)/E(s) = Kp + Ki/s + Kd s
    L(s)  = (Kp s + Ki + Kd s^2)/s

Okay, So the error is multiplied to P, I and D by error E = Set point - Procress variable

Setpoint = target value
Procress variable = current measure value of the procress; ex current measure value of temperature of furnance

E = SP - PV
P = Proportional Gain / Proportional Band/ Kp (How fast the system responses)
Adjusting this term higher may cause sensetive less sensetive loops.
Ex if the elevator of aircraft has high Kp need to maintain 2500. Then
As soon as Auto Pilot engages the Angle of Attack will be high, even if some sort of AOA protection and prevents that,
it might oversoot 2500 ft margin.

Decreasing this parameter might make the control response slow.

Interal term: I constant. Ki/ Ti
The integral is the sum of all of the values reported from the signal captured from
when we started counting to end where the area under the plotted curve.
This term determines how fast the steady error is removed.

If Ki is too large error accumulates fast makes controller too agressive in fixing past errors
which causes, overshoot, oscillation.
Oscillation: ex aircraft has 2000ft in selected mode it will overshoot goes for 2300 ft and again go down more and be unstable.

Derivative are seconds or minutes. D = PV(t2) - PV(t1) / t2 - t1
It means how far in the future you want to predict the change.
Signal should be clean for derivative controller.
It's basically how fast the error is changing.

Okay let's say I am at 1700 ft for 2000ft in an aircraft
if the derivative term says the rate of change is e(t) = 2000-1700 = 300ft/min
If I am at 2000 ft and still vertical speed is like 500ft/min I am apporaching too fast.

This meant to be like a Note, if I ever need this thing in my life again I will look at it.
*/

typedef struct
{
    float KP;
    float KI;
    float KD;
    int testIteration;
} Constant;

/* After experimenting for a day, running this C program making a csv value and plotting from R,
   Since we don't have a sensor value we need to simulate one. A real sensor is always noisy, there
   are different parameters affeting it like temperature, altitude, preassure etc. Wow acutally I am learning a lot today.
*/

// Genterating random noise, rand() gives uniform random number between 0 and RAND_MAX
double random_values()
{
    // rand() gives 0 to RAND_MAX make sure it's not 0 and RAND_MAX is the maximum value that it gives and we scale it between 0 to 1,
    // log 0 breakes Box-Muller formula, we need to avoid that, 1/RAND_MAX breaks the Box-Muller transformation so we add 2
    return (rand() + 1.0) / (RAND_MAX + 2.0);
}

double simulate_sensor(double true_value)
{ // true value of estimation

    double u1 = random_values();
    double u2 = random_values();

    // Box–Muller transform the famous bell curve, we need value spread out about its mean
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    // 0.05 is the flucuation that the sensor might read, okay if a GPWS sensor in aircraft calls 1000ft the fluctuation might be 0.05 ft
    // 0.01 is the small noise
    double noise_std = 0.05 * fabs(true_value) + 0.001; // standard deviation of the Gaussian noise, std dev always tells us how spread out the values are around the mean
    return true_value + z0 * noise_std;
    // example, true value is 25, noise 1.26 and z0 -0.3
}

/* Since it's a closed system. It needs to simulate how a real system (that we refer to as a plant)
First order dynamic system
react to a control input over a small time dt., If the true_value is just a constant we won't get the bell curve.
terminology: plant = ex 20ft current state,
             control -> input from a controller
             tau_plant -> refers how fast the plant reacts
             dx -> rate of change of the plant at this moment

dx/dt = -x  ---i)
Euler method
x (k + 1) = xk + f(xk, tk). del t
in the above sol f(x,t) = -x;
x(k + 1) = xk + (-xk) . del t
x(k + 1) = xk - xk del t
x (k + 1) = xk (1-delt) ---- ii)

Okay from what I learned in school to solve this,
intregate on both sides,
 ∫ dx/x = ∫ - dt
 ln|x| = -t + C
 x(t) = C e^-t  solve it iteratively and check the value it's an approximation

 at initial condition x(0) = C = xo
 x(t) = xo e^-t ---- iii)
 eqn ii) is the approximation of eqn iii)

Eqn d(plant)/dt = -plant + contorl / tau_plant,  double dx = (-plant + control) / tau_plant;
if a differential eqn,
Euler integration note:- Takes current values moves a small step in the different direction of slope repeat.

x_{k+1} = x_k + dx * dt = plant += dx * dt;  not good at maths but for the ref here it is after Euler method.
*/

double simulate_plant(double plant, double control, double dt, double tau_plant)
{
    // since the plant is not that responsive let's change the value of our tau_plane that came from solving the Euler method, differential equation
    double dx = (-plant + control) / tau_plant;
    plant += dx * dt;
    return plant;
}

double clamp(double value, double min, double max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

int simple_pid(Constant k, float setPoint)
{
    float previous_error = 0.0f;
    float integral_sum = 0.0f;
    double plant = 0.0f;

    FILE *ptr;
    ptr = fopen("data.csv", "w");

    struct timespec prev_time;
    clock_gettime(CLOCK_MONOTONIC, &prev_time);
    int c = 0;
    double plant_tau = 3.0; // plant time contant in sec

    fprintf(ptr, "Delta,Setpoint,Output,Sim,Plant,P,I,D\n");
    while (1)
    {

        if (c > k.testIteration)
            break;
        // I have faced a problem that couldn't be fixed for a day now let's analyze step by step what happened
        // I haven't use C ever since school, so small mitake can make a huge difference here

        // escaped_time
        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        float dt = (current_time.tv_sec - prev_time.tv_sec) +
                   (current_time.tv_nsec - prev_time.tv_nsec) / 1e9f;

        double measurement_sensor = simulate_sensor(plant);

        /* tv_sec whole seconds, tv_nsec nano second, diff in whole seconds + difference in nano seconds / 1e9f convers ns to s,
        we saw huge value meaning derviative term is divided by a small number */

        // okay we have error here
        float delta = setPoint - measurement_sensor;

        // for p_term
        float proportioanl_term = k.KP * delta;

        // for derivative term, line 40 for ref
        // if dt is too small then derivative term explodes, similar to vanishing gradient issue that we can across in Deep Nerual Network
        float derivative_term = 0.0f;
        if (c > 0 && dt >= 1e-6f) // let's try to skip derivative term in first iteration, dejavu from neural net project where I forgot to activate and gradient vanished
            derivative_term = k.KD * (delta - previous_error) / dt;

        // for interagal term, sums of all the error from start
        integral_sum += delta * dt;

        float output_raw_check = proportioanl_term + (k.KI * integral_sum) + derivative_term;
        // we can't intrage error so higher than the physical limit of the device
        if (output_raw_check > OUTPUT_MAX)
        {
            if (delta > 0.0f)
                integral_sum -= delta * dt;
        }
        else if (output_raw_check < OUTPUT_MIN)
        {
            if (delta < 0.0f)
                integral_sum -= delta * dt;
        }

        float integral_term = k.KI * integral_sum;

        float output = proportioanl_term + integral_term + derivative_term;
        output = clamp(output, OUTPUT_MIN, OUTPUT_MAX); // it forces value to be withing specified range
        // when I just kept the random value it was unstable

        plant = simulate_plant(plant, output, dt, plant_tau); // ex its the deflection of aileron in case of aircraft
        fprintf(ptr, "%.2f,%.2f,%.2f,%f,%f,%f,%f,%f\n", delta, setPoint, output, measurement_sensor, plant, proportioanl_term, integral_term, derivative_term);
        previous_error = delta;
        prev_time = current_time;

        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 100 * 1000000L;
        nanosleep(&ts, NULL); // wait for 100ms
        c++;
    }
    fclose(ptr);
    return 0;
}

int main()
{
    Constant k;
    k.KP = 0.01f;
    k.KI = 0.1f;
    k.KD = 0.09f;
    k.testIteration = 300;
    simple_pid(k, 10.0f);
    /* Imagine if you're on your car and throtle is full for a sec will that make a difference,
    considering it's mass and interia it would take to acclerate, no but we want the system to be optimal */

    /* With these tuning paramaters there is little bit of a ostillation in our controller, aslo the noise from the 
    sensor is high compared to the reposne from our plant.
    
    The sensor is not that noisy but the way we are visualizing it makes it look like the sensor is noisy. */
    return 0;
}