#ifndef FILTER_BPF
#define FILTER_BPF

class Filter_BPF{
    public:
        Filter_BPF();

        void set_param();
        void get_param();

        float get_y();

        void cal_y(float x_i);
        void reset();

    private:
};

#endif