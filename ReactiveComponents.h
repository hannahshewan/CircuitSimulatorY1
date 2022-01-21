#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <complex>

using namespace std::complex_literals;

struct Capacitor : Component {
    double capacitance;

    Capacitor(int nodes_in [2], double capacitance_in, bool debug=false){
        component_type = ComponentType::Capacitor;
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        capacitance = capacitance_in;

        if(debug)
            std::cout << "\t Creating capacitor with capacitance " << capacitance << std::endl;
    }

    void fill_matrices_DC(Matrices<double>& matrices) override{
        //std::cout << " Adding capacitor to matrices with capacitance " << capacitance << std::endl;
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double ang_freq) override{
        
        resize_node_matrices(matrices, nodes, no_nodes);

        std::complex<double> admittance = ang_freq * capacitance * 1i;

        //std::cout << "Adding capacitor to matrices with capacitance " << capacitance << " and admittance " << admittance << std::endl;

        add_element_cond(matrices.G_matrix, nodes[0] - 1, nodes[0] - 1, admittance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, nodes[0] - 1, -admittance);
        add_element_cond(matrices.G_matrix, nodes[0] - 1, nodes[1] - 1, -admittance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, nodes[1] - 1, admittance);


    }

};

struct Inductor : Component {
    double inductance;

    Inductor(int nodes_in [2], double inductance_in, bool debug=false){
        component_type = ComponentType::Inductor;
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        inductance = inductance_in;
        if (debug)
            std::cout << "\t Creating inductor with inductance " << inductance << std::endl;
    }

    void fill_matrices_DC(Matrices<double>& matrices) override{
        //std::cout << "Adding inductance " << inductance << std::endl;
        
        DCVoltageSource* vs = new DCVoltageSource(nodes, 0);
        vs->fill_matrices_DC(matrices);
        delete(vs);
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double ang_freq) override{
        //std::cout << "Adding inductance source inductance " << inductance << std::endl;
        
        std::complex<double> admittance = 1.0/(ang_freq * inductance * 1i);
        add_element_cond(matrices.G_matrix, nodes[0] - 1, nodes[0] - 1, admittance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, nodes[0] - 1, -admittance);
        add_element_cond(matrices.G_matrix, nodes[0] - 1, nodes[1] - 1, -admittance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, nodes[1] - 1, admittance);
    }
};