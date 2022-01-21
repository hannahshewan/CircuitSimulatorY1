#include <vector>
#include <Eigen/Dense>
#include <cmath>

struct Diode : Component {
    double sat_current;
    double thermal_volt;

    Diode(int nodes_in [2], bool debug=false) {
        component_type = ComponentType::Diode;

        //0 index node is anode, 1 index node is cathode
        //current in diode goes from anode (+) to cathode (-)
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        sat_current = 10E-15;
        thermal_volt = 25E-3;

        if(debug)
            std::cout << "\t Created a diode between nodes " << nodes[0] << " " << nodes[1] << std::endl;

    }

    //fill conductance (g) and current matrix (i) by linearilising the diode 
    //a voltage between the first node and the second

    void get_eq_circuit(long double volt_diff_guess, long double& eq_cond, long double& eq_curr){
        long double i_v0 = sat_current * ( pow(2.71828, volt_diff_guess/thermal_volt) - 1);
        long double i_diff_v0 = sat_current/thermal_volt * pow(2.71828, volt_diff_guess/thermal_volt);

        if(volt_diff_guess > 0){ //forward bias
            
            eq_cond = i_diff_v0 /*+ 0.001*/;
            eq_curr = i_v0 - i_diff_v0 * volt_diff_guess /*- sat_current + volt_diff_guess * 0.001*/;
        } else { // reverse bias
            //std::cout << "Diode is in reverse bias " << std::endl;
            eq_cond = 0.000001;
            eq_curr = -sat_current + volt_diff_guess * 0.000001;
        }

        if(eq_cond == NAN || eq_cond == INFINITY || eq_curr == NAN || eq_curr == INFINITY){
            std::cout << "Error - Encountered NaN or INF while getting equivalent circuit for diode between nodes " << nodes[0] << " & " << nodes[1] << " not added." << std::endl;
            //For now I have disabled the early return - I want to it to be clear to me if something goes wrong
            //return;
        }
    }

    void fill_matrices_DC(Matrices<double>& matrices, const Eigen::VectorXd& nodal_guesses) override {
                
        //std::cout << "Filling matrices with diode. Nodes connected are " << nodes[0] << "  " << nodes[1] << std::endl;
        
        long double volt_diff_guess = get_element_cond(nodal_guesses, nodes[0] - 1) - get_element_cond(nodal_guesses, nodes[1] - 1);

        //std::cout << "volt diff guess: " << volt_diff_guess << std::endl;

        long double eq_cond;
        long double eq_curr;


        get_eq_circuit(volt_diff_guess, eq_cond, eq_curr);

        int curr_nodes [2];
        curr_nodes[0] = nodes[1];
        curr_nodes[1] = nodes[0];

        DCCurrentSource* eq_current_src = new DCCurrentSource(curr_nodes, eq_curr);
        eq_current_src->fill_matrices_DC(matrices);
        delete eq_current_src;

        Resistor* eq_resistor = new Resistor(nodes, eq_cond);
        eq_resistor->fill_matrices_DC(matrices);
        delete eq_resistor;
        //return 0;  
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double ang_freq) override {
                
        //std::cout << "Filling matrices with diode. Nodes connected are " << nodes[0] << "  " << nodes[1] << std::endl;
        
        long double volt_diff_guess = get_element_cond(nodal_voltages, nodes[0] - 1) - get_element_cond(nodal_voltages, nodes[1] - 1);

        long double eq_cond, eq_curr;

        get_eq_circuit(volt_diff_guess, eq_cond, eq_curr);

        int curr_nodes [2];
        curr_nodes[0] = nodes[1];
        curr_nodes[1] = nodes[0];

        Resistor* eq_resistor = new Resistor(nodes, eq_cond);
        eq_resistor->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete eq_resistor;
        //return 0;  
    }


};