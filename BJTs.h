#include <vector>
#include <Eigen/Dense>
#include <cmath>

struct NPN_BJT : Component {
    double i_s;
    double v_t;
    double v_a;
    double beta;
    double reverse_beta;

    int collector;
    int base;
    int emitter;

    // 0: Collector, 1: Base, 2: Emitter 
    NPN_BJT(int nodes_in [3], double beta_in=200, std::string model_name=""){
        component_type = ComponentType::NPN_BJT;

        collector = nodes[0] = nodes_in[0];
        base = nodes[1] = nodes_in[1];
        emitter = nodes[2] = nodes_in[2];

        i_s = 10E-14;
        v_t = 25E-3;
        v_a = 400;
        beta = beta_in;
        reverse_beta = 1;
    }

    void fill_matrices_DC(Matrices<double>& matrices, const Eigen::VectorXd& volt_guesses) override {
        
        double collector_volt = get_element_cond(volt_guesses, nodes[0] - 1);
        double base_volt = get_element_cond(volt_guesses, nodes[1] - 1);
        double emitter_volt = get_element_cond(volt_guesses, nodes[2] - 1);

        double v_be = base_volt - emitter_volt;
        double v_ce = collector_volt - emitter_volt;
        double v_bc = base_volt - collector_volt;


        std::cout << "collector: " << collector << " base: " << base << " emitter" << emitter << std::endl;
        std::cout << " v_be: " << v_be << " v_ce: " << v_ce << " v_bc: " << v_bc << std::endl;

        if(-v_bc > 0 && v_be > 0){
            std::cout << "BJT is in active mode " << std::endl;
        } else if(-v_bc < 0 && v_be > 0){
            std::cout << "BJT is in saturation" << std::endl;
        } else if(-v_bc < 0 && v_be < 0){
            std::cout << "BJT is in reverse-active" << std::endl;
        } else {
            std::cout << "BJT is in cut-off" << std::endl;
        }

        // i_bf
        double i_be = (i_s/beta) * (pow(2.71828, v_be/v_t) - 1.0);
        double i_bc = (i_s/reverse_beta) * (pow(2.71828, v_bc/v_t) - 1.0);

        std::cout << "i_be " << i_be << " i_bc " << i_bc << std::endl;


        //diode between base and emitter
        double be_eq_cond = i_be / v_t;
        //be_eq_cond = be_eq_cond > 0 ? be_eq_cond : 0.0001;
        double be_eq_current = i_be - be_eq_cond*v_be;
        
        int emitter_base_nodes [2] = {emitter, base};
        Resistor* rs_be = new Resistor(emitter_base_nodes, be_eq_cond, true);
        rs_be->fill_matrices_DC(matrices);
        delete(rs_be);

        DCCurrentSource* cs_be = new DCCurrentSource(emitter_base_nodes, be_eq_current);
        cs_be->fill_matrices_DC(matrices);
        delete(cs_be);

        //diode between base and collector
        double bc_eq_cond = i_bc / v_t;
        //bc_eq_cond = bc_eq_cond > 0 ? bc_eq_cond : 0.0001;

        double bc_eq_current = i_bc - bc_eq_cond*v_bc;

        int collector_base_nodes [2] = {collector, base};
        Resistor* rs_bc = new Resistor(collector_base_nodes, bc_eq_cond, true);
        rs_bc->fill_matrices_DC(matrices);
        delete(rs_bc);

        DCCurrentSource* cs_bc = new DCCurrentSource(collector_base_nodes, bc_eq_current);
        cs_bc->fill_matrices_DC(matrices);
        delete(cs_bc);

        //be term here means that the transconductance is dependent on v_be
        double be_eq_transconductance = beta * be_eq_cond;
        double bc_eq_transconductance = reverse_beta * bc_eq_cond;
        
        int emitter_collector_nodes [2] = {emitter, collector};
        DependentCurrentSource* dcs_be = new DependentCurrentSource(emitter_collector_nodes, emitter_base_nodes, be_eq_transconductance);
        dcs_be->fill_matrices_DC(matrices);
        delete(dcs_be);

        int collector_emitter_nodes [2] = {collector, emitter};
        DependentCurrentSource* dcs_bc = new DependentCurrentSource(collector_emitter_nodes, collector_base_nodes, bc_eq_transconductance);
        dcs_bc->fill_matrices_DC(matrices);
        delete(dcs_bc);

        double I_c = i_s * pow(2.71828, v_be/v_t)*(1.0+v_ce/v_a);
        
        double ce_eq_cond = I_c/v_a;
        //ce_eq_cond = ce_eq_cond > 0 ? ce_eq_cond : 0.0001;
        Resistor* rs_ce = new Resistor(collector_emitter_nodes, ce_eq_cond , true);
        rs_ce->fill_matrices_DC(matrices);
        delete(rs_ce);


        double I_c_eq =  I_c
                        - be_eq_transconductance * v_be
                        + bc_eq_transconductance * v_bc
                        - ce_eq_cond * v_ce;

        //std::cout << "I_c: " << I_c << " I_c_eq: " << I_c_eq << std::endl;

        DCCurrentSource* cs_ce = new DCCurrentSource(emitter_collector_nodes, I_c_eq);
        cs_ce->fill_matrices_DC(matrices);
        delete(cs_ce);

    }

    virtual void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) {
        double collector_volt = get_element_cond(nodal_voltages, nodes[0] - 1);
        double base_volt = get_element_cond(nodal_voltages, nodes[1] - 1);
        double emitter_volt = get_element_cond(nodal_voltages, nodes[2] - 1);

        double v_be = base_volt - emitter_volt;
        double v_ce = collector_volt - emitter_volt;
        double v_bc = base_volt - collector_volt;


        std::cout << "collector: " << collector << " base: " << base << " emitter" << emitter << std::endl;
        std::cout << " v_be: " << v_be << " v_ce: " << v_ce << " v_bc: " << v_bc << std::endl;

        if(-v_bc > 0 && v_be > 0){
            std::cout << "BJT is in active mode " << std::endl;
        } else if(-v_bc < 0 && v_be > 0){
            std::cout << "BJT is in saturation" << std::endl;
        } else if(-v_bc < 0 && v_be < 0){
            std::cout << "BJT is in reverse-active" << std::endl;
        } else {
            std::cout << "BJT is in cut-off" << std::endl;
        }

        // i_bf
        double i_be = (i_s/beta) * (pow(2.71828, v_be/v_t) - 1.0);
        double i_bc = (i_s/reverse_beta) * (pow(2.71828, v_bc/v_t) - 1.0);

        std::cout << "i_be " << i_be << " i_bc " << i_bc << std::endl;


        //diode between base and emitter
        double be_eq_cond = i_be / v_t;
        //be_eq_cond = be_eq_cond > 0 ? be_eq_cond : 0.0001;
        double be_eq_current = i_be - be_eq_cond*v_be;
        
        int emitter_base_nodes [2] = {emitter, base};
        Resistor* rs_be = new Resistor(emitter_base_nodes, be_eq_cond, true);
        rs_be->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(rs_be);

        //diode between base and collector
        double bc_eq_cond = i_bc / v_t;
        //bc_eq_cond = bc_eq_cond > 0 ? bc_eq_cond : 0.0001;

        double bc_eq_current = i_bc - bc_eq_cond*v_bc;

        int collector_base_nodes [2] = {collector, base};
        Resistor* rs_bc = new Resistor(collector_base_nodes, bc_eq_cond, true);
        rs_bc->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(rs_bc);

        //be term here means that the transconductance is dependent on v_be
        double be_eq_transconductance = beta * be_eq_cond;
        double bc_eq_transconductance = reverse_beta * bc_eq_cond;
        
        int emitter_collector_nodes [2] = {emitter, collector};
        DependentCurrentSource* dcs_be = new DependentCurrentSource(emitter_collector_nodes, emitter_base_nodes, be_eq_transconductance);
        dcs_be->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(dcs_be);

        int collector_emitter_nodes [2] = {collector, emitter};
        DependentCurrentSource* dcs_bc = new DependentCurrentSource(collector_emitter_nodes, collector_base_nodes, bc_eq_transconductance);
        dcs_bc->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(dcs_bc);

        double I_c = i_s * pow(2.71828, v_be/v_t)*(1.0+v_ce/v_a);
        
        double ce_eq_cond = I_c/v_a;
        //ce_eq_cond = ce_eq_cond > 0 ? ce_eq_cond : 0.0001;
        Resistor* rs_ce = new Resistor(collector_emitter_nodes, ce_eq_cond , true);
        rs_ce->fill_matrices_AC(matrices, nodal_voltages, 0);
        delete(rs_ce);


        double I_c_eq =  I_c
                        - be_eq_transconductance * v_be
                        + bc_eq_transconductance * v_bc
                        - ce_eq_cond * v_ce;

        std::cout << "I_c: " << I_c << " I_c_eq: " << I_c_eq << std::endl;
    }

};