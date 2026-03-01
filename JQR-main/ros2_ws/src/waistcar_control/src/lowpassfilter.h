# include <iostream>
# include <cmath>

class lowpassfilter {
public:
    lowpassfilter()
    {
        alpha = 1;
        state = 0;
    }

    void setparams(double sample_rate, double cutoff_freq, double init_input)
    {
        double dt = 1.0 / sample_rate;
        double rc = 1.0 / (cutoff_freq * 2.0 * M_PI);
        alpha = dt / (dt + rc);
        state = init_input;
    }

    void reset()
    {
        state = 0.0;
    }

    double update(double input)
    {
        double output = alpha*input + (1.0 - alpha) * state;
        state = output;
        return output;
    }

    double alpha;
    double state;

private:

};