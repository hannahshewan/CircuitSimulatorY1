#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>


struct NMOS : Component {

    double v_t;
    double K;
    double v_A;

    //0: Drain, 1: Gate, 2: Source
    NMOS(int nodes_in [3], double v_t_in=1, double K_in=1, double v_A_in=400, bool debug=false) : v_t(v_t_in), K(K_in), v_A(v_A_in) {
        
        //default values are based off Si7336ADP
        
        component_type = ComponentType::NMOS;
        
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        nodes[2] = nodes_in[2];

        //K is the same as k'p(W/L)
        //k'p is kp(L/W)

        if(debug)
            std::cout << "\t Creating NMOS with the following nodes: Drain: " << nodes[0] << " Gate: " << nodes[1] << " Source: " << nodes[2] << std::endl;
    }

    void fill_matrices_DC(Matrices<double>& matrices, const Eigen::VectorXd& nodal_guesses) override{
        //std::cout << "Filling matrices with NMOS. Drain: " << nodes[0] << " Gate: " << nodes[1] << " Source: " << nodes[2] << std::endl;

        double drain_volt = get_element_cond(nodal_guesses, nodes[0] - 1);
        double gate_volt = get_element_cond(nodal_guesses, nodes[1] - 1);
        double source_volt = get_element_cond(nodal_guesses, nodes[2] - 1);

        double v_ds = drain_volt - source_volt;
        double v_gs = gate_volt - source_volt;

        //drain source equivalent parameters for resistor, vccs and cs
        double eq_cond = 0; //partial derivative of id wrt v_ds
        double eq_transconductance = 0; //partial derivative of id wrt v_gs
        double eq_current = 0;

        // the drain current is well-defined in all modes of operation
        // and this current should always hold true
        // in other words the current through the equivalent circuit should be equal to this
        double desired_I_d = 0;


        //std::cout << "v_ds: " << v_ds << " v_gs: " << v_gs << std::endl;

        if(v_gs >= v_t){ // make sure we are not in cut-off
            if(v_ds >= v_gs - v_t){ //saturation
                //std::cout << "NMOS is in saturation" << std::endl;
                desired_I_d = K/2 * pow(v_gs - v_t, 2)  * (1.0 + (v_ds-v_gs-v_t)/v_A);
                
                //I_d partial derivatve wrt to vgs
                eq_transconductance = K*(v_gs - v_t) *(1.0+(v_ds-v_gs-v_t)/v_A);

                //I_d partial derivatve wrt to vds
                eq_cond = K/2 * pow(v_gs - v_t, 2) / v_A;

                eq_current = desired_I_d - eq_cond*v_ds - eq_transconductance*v_gs;
                //std::cout << eq_current << " " << eq_transconductance << " " << desired_I_d << std::endl;

            } else { //triode
                //std::cout << "NMOS is in triode" << std::endl;
                //Id = K * [ 2(v_gs - v_t)*v_ds - v_ds^2]
                // dId/d_DS = 2K * [ v_gs - v_t - v_ds]

                desired_I_d = K * ((v_gs - v_t) - v_ds/2)*v_ds;
                
                //I_d partial derivatve wrt to vgs
                eq_transconductance = K*v_ds;

                //I_d partial derivatve wrt to vds
                eq_cond = K * ( (v_gs - v_t) - v_ds);

                eq_current = desired_I_d - eq_cond*v_ds - eq_transconductance*v_gs;

            }

        } else {
            //std::cout << "NMOS is in cut-off" << std::endl;
            //ds_eq_curr = 10E-14 - v_ds * 0.001;
            //ds_eq_cond = 0.0001;
            eq_cond = 0.0001;
            
            //ds_eq_curr = -sat_current + volt_diff_guess * 0.001;
        }

        
        //MOSFET can be modelled with a dependent current (on vgs), independent current source,
        // and a 

        int eq_nodes [2];
        eq_nodes[0] = nodes[2]; //source
        eq_nodes[1] = nodes[0]; //drain

        int control_nodes [2];
        control_nodes[0] = nodes[2]; //source
        control_nodes[1] = nodes[1]; //gate
        
        if(v_gs >= v_t){
            DependentCurrentSource* eq_dpcurrent_src = new DependentCurrentSource(eq_nodes, control_nodes, eq_transconductance);
            eq_dpcurrent_src->fill_matrices_DC(matrices);
            delete(eq_dpcurrent_src);

            DCCurrentSource* eq_current_src = new DCCurrentSource(eq_nodes, eq_current);
            eq_current_src->fill_matrices_DC(matrices);
            delete(eq_current_src);
        }
        
        Resistor* eq_resistor = new Resistor(eq_nodes, eq_cond);
        eq_resistor->fill_matrices_DC(matrices);
        delete(eq_resistor);

    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) override{
        //std::cout << "Filling matrices with NMOS. Drain: " << nodes[0] << " Gate: " << nodes[1] << " Source: " << nodes[2] << std::endl;

        double drain_volt = get_element_cond(nodal_voltages, nodes[0] - 1);
        double gate_volt = get_element_cond(nodal_voltages, nodes[1] - 1);
        double source_volt = get_element_cond(nodal_voltages, nodes[2] - 1);

        double v_ds = drain_volt - source_volt;
        double v_gs = gate_volt - source_volt;

        //drain source equivalent parameters for resistor, vccs and cs
        double eq_cond = 0; //partial derivative of id wrt v_ds
        double eq_transconductance = 0; //partial derivative of id wrt v_gs
        double eq_current = 0;

        // the drain current is well-defined in all modes of operation
        // and this current should always hold true
        // in other words the current through the equivalent circuit should be equal to this
        double desired_I_d = 0;


        //std::cout << "v_ds: " << v_ds << " v_gs: " << v_gs << std::endl;

        if(v_gs >= v_t){ // make sure we are not in cut-off
            if(v_ds >= v_gs - v_t){ //saturation
                //std::cout << "NMOS is in saturation" << std::endl;
                desired_I_d = K/2 * pow(v_gs - v_t, 2) * (1.0 + (v_ds-v_gs-v_t)/v_A);
                
               //I_d partial derivatve wrt to vgs
                eq_transconductance = K*(v_gs - v_t)*(1.0+(v_ds-v_gs-v_t)/v_A);

                 //I_d partial derivatve wrt to vds
                eq_cond = K/2 * pow(v_gs - v_t, 2) / v_A;

                eq_current = desired_I_d - eq_cond*v_ds - eq_transconductance*v_gs;
                

            } else { //triode
                //std::cout << "NMOS is in triode" << std::endl;
                //Id = K * [ 2(v_gs - v_t)*v_ds - v_ds^2]
                // dId/d_DS = 2K * [ v_gs - v_t - v_ds]

                desired_I_d = K * ((v_gs - v_t) - v_ds/2)*v_ds;
                
                //I_d partial derivatve wrt to vgs
                eq_transconductance = K*v_ds;

                //I_d partial derivatve wrt to vds
                eq_cond = K * ( (v_gs - v_t) - v_ds);

                eq_current = desired_I_d - eq_cond*v_ds - eq_transconductance*v_gs;

            }

        } else {
            //std::cout << "NMOS is in cut-off" << std::endl;
            //ds_eq_curr = 10E-14 - v_ds * 0.001;
            //ds_eq_cond = 0.0001;
            eq_cond = 0.0001;
            
            //ds_eq_curr = -sat_current + volt_diff_guess * 0.001;
        }

        
        //MOSFET can be modelled with a dependent current (on vgs), independent current source,
        // and a 

        int eq_nodes [2];
        eq_nodes[0] = nodes[2]; //source
        eq_nodes[1] = nodes[0]; //drain

        int control_nodes [2];
        control_nodes[0] = nodes[2]; //source
        control_nodes[1] = nodes[1]; //gate
        
        if(v_gs >= v_t){
            DependentCurrentSource* eq_dpcurrent_src = new DependentCurrentSource(eq_nodes, control_nodes, eq_transconductance);
            eq_dpcurrent_src->fill_matrices_AC(matrices, nodal_voltages, 0);
            delete(eq_dpcurrent_src);
        }
        
        Resistor* eq_resistor = new Resistor(eq_nodes, eq_cond, true);
        eq_resistor->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(eq_resistor);
    }


};

struct PMOS : Component {

    double v_t;
    double K;
    double v_A;

    //0: Drain, 1: Gate, 2: Source
    PMOS(int nodes_in [3], double v_t_in=1, double K_in=0.0018, double v_A_in=400, bool debug=false) : v_t(v_t_in), K(K_in), v_A(v_A_in) {
                
        component_type = ComponentType::PMOS;
        
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        nodes[2] = nodes_in[2];

        if(debug)
            std::cout << "\t Creating NMOS with the following nodes: Drain: " << nodes[0] << " Gate: " << nodes[1] << " Source: " << nodes[2] << std::endl;
    }

    void fill_matrices_DC(Matrices<double>& matrices, const Eigen::VectorXd& nodal_guesses) override{
        //std::cout << "Filling matrices with PMOS. Drain: " << nodes[0] << " Gate: " << nodes[1] << " Source: " << nodes[2] << std::endl;

        double drain_volt = get_element_cond(nodal_guesses, nodes[0] - 1);
        double gate_volt = get_element_cond(nodal_guesses, nodes[1] - 1);
        double source_volt = get_element_cond(nodal_guesses, nodes[2] - 1);

        double v_sd = source_volt - drain_volt;
        double v_sg = source_volt - gate_volt;

        //drain source equivalent parameters for resistor, vccs and cs
        double eq_cond = 0; //partial derivative of id wrt v_ds
        double eq_transconductance = 0; //partial derivative of id wrt v_gs
        double eq_current = 0;

        // the drain current is well-defined in all modes of operation
        // and this current should always hold true
        // in other words the current through the equivalent circuit should be equal to this
        double desired_I_s = 0;


        //std::cout << "v_sd: " << v_sd << " v_sg: " << v_sg << std::endl;



        if(v_sg > abs(v_t)){ // make sure we are not in cut-off
            if(v_sd > v_sg - abs(v_t)){ //saturation
                //std::cout << "PMOS is in saturation" << std::endl;
                desired_I_s = K/2 * pow(v_sg - v_t, 2) * (1.0 + (v_sd-v_sg-v_t)/v_A);
                
               //I_d partial derivatve wrt to vgs
                eq_transconductance = K*(v_sg - v_t)*(1.0+(v_sd-v_sg-v_t)/v_A);

                 //I_d partial derivatve wrt to vds
                eq_cond = K/2 * pow(v_sg - v_t, 2) / v_A;

                eq_current = desired_I_s - eq_cond*v_sd - eq_transconductance*v_sg;
                

            } else { //triode
                //std::cout << "PMOS is in triode" << std::endl;
                //Id = K * [ 2(v_gs - v_t)*v_ds - v_ds^2]
                // dId/d_DS = 2K * [ v_gs - v_t - v_ds]

                desired_I_s = K * ((v_sg - v_t) - v_sd/2)*v_sd;
                
                //I_d partial derivatve wrt to vgs
                eq_transconductance = K*v_sd;

                //I_d partial derivatve wrt to vds
                eq_cond = K * ( (v_sg - v_t) - v_sd);

                eq_current = desired_I_s - eq_cond*v_sd - eq_transconductance*v_sg;

            }

        } else {
            //std::cout << "PMOS is in cut-off" << std::endl;
            //ds_eq_curr = 10E-14 - v_ds * 0.001;
            //ds_eq_cond = 0.0001;
            eq_cond = 0.0001;
            
            //ds_eq_curr = -sat_current + volt_diff_guess * 0.001;
        }

        
        //MOSFET can be modelled with a dependent current (on vgs), independent current source,
        // and a 

        //reversed nodes for PMOS
        int eq_nodes [2];
        eq_nodes[0] = nodes[0];
        eq_nodes[1] = nodes[2]; 

        int control_nodes [2];
        control_nodes[0] = nodes[1]; 
        control_nodes[1] = nodes[2]; 
        
        if(v_sg > abs(v_t)){
            DependentCurrentSource* eq_dpcurrent_src = new DependentCurrentSource(eq_nodes, control_nodes, eq_transconductance);
            eq_dpcurrent_src->fill_matrices_DC(matrices);
            delete(eq_dpcurrent_src);

            DCCurrentSource* eq_current_src = new DCCurrentSource(eq_nodes, eq_current);
            eq_current_src->fill_matrices_DC(matrices);
            delete(eq_current_src);
        }
        
        Resistor* eq_resistor = new Resistor(eq_nodes, eq_cond);
        eq_resistor->fill_matrices_DC(matrices);
        delete(eq_resistor);
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double ang_freq) override{
        std::cout << "Filling matrices with PMOS. Drain: " << nodes[0] << " Gate: " << nodes[1] << " Source: " << nodes[2] << std::endl;

        double drain_volt = get_element_cond(nodal_voltages, nodes[0] - 1);
        double gate_volt = get_element_cond(nodal_voltages, nodes[1] - 1);
        double source_volt = get_element_cond(nodal_voltages, nodes[2] - 1);

        double v_sd = source_volt - drain_volt;
        double v_sg = source_volt - gate_volt;

        //drain source equivalent parameters for resistor, vccs and cs
        double eq_cond = 0; //partial derivative of id wrt v_ds
        double eq_transconductance = 0; //partial derivative of id wrt v_gs
        double eq_current = 0;

        // the drain current is well-defined in all modes of operation
        // and this current should always hold true
        // in other words the current through the equivalent circuit should be equal to this
        double desired_I_s = 0;


        //std::cout << "v_sd: " << v_sd << " v_sg: " << v_sg << std::endl;



        if(v_sg > abs(v_t)){ // make sure we are not in cut-off
            if(v_sd > v_sg - abs(v_t)){ //saturation
                //std::cout << "PMOS is in saturation" << std::endl;
                desired_I_s = K/2 * pow(v_sg - v_t, 2) * (1.0 + (v_sd-v_sg-v_t)/v_A);
                
               //I_d partial derivatve wrt to vgs
                eq_transconductance = K*(v_sg - v_t)*(1.0+(v_sd-v_sg-v_t)/v_A);

                 //I_d partial derivatve wrt to vds
                eq_cond = K/2 * pow(v_sg - v_t, 2) / v_A;

                eq_current = desired_I_s - eq_cond*v_sd - eq_transconductance*v_sg;
                

            } else { //triode
                //std::cout << "PMOS is in triode" << std::endl;
                //Id = K * [ 2(v_gs - v_t)*v_ds - v_ds^2]
                // dId/d_DS = 2K * [ v_gs - v_t - v_ds]

                desired_I_s = K * ((v_sg - v_t) - v_sd/2)*v_sd;
                
                //I_d partial derivatve wrt to vgs
                eq_transconductance = K*v_sd;

                //I_d partial derivatve wrt to vds
                eq_cond = K * ( (v_sg - v_t) - v_sd);

                eq_current = desired_I_s - eq_cond*v_sd - eq_transconductance*v_sg;

            }

        } else {
            //std::cout << "PMOS is in cut-off" << std::endl;
            //ds_eq_curr = 10E-14 - v_ds * 0.001;
            //ds_eq_cond = 0.0001;
            eq_cond = 0.0001;
            
            //ds_eq_curr = -sat_current + volt_diff_guess * 0.001;
        }

        
        //MOSFET can be modelled with a dependent current (on vgs), independent current source,
        // and a 

        //reversed nodes for PMOS
        int eq_nodes [2];
        eq_nodes[0] = nodes[0];
        eq_nodes[1] = nodes[2]; 

        int control_nodes [2];
        control_nodes[0] = nodes[1]; 
        control_nodes[1] = nodes[2]; 
        
        if(v_sg > abs(v_t)){
            DependentCurrentSource* eq_dpcurrent_src = new DependentCurrentSource(eq_nodes, control_nodes, eq_transconductance);
            eq_dpcurrent_src->fill_matrices_AC(matrices, nodal_voltages, 0);
            delete(eq_dpcurrent_src);
        }
        
        Resistor* eq_resistor = new Resistor(eq_nodes, eq_cond, true);
        eq_resistor->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(eq_resistor);
    }
};