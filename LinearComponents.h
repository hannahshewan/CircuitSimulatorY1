#include <vector>
#include <Eigen/Dense>
#include <cmath>

struct Resistor : Component {
    double conductance;

    Resistor(int nodes_in [2], double conductance_in, bool debug=false){
        component_type = ComponentType::Resistor;
        
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];

        conductance = conductance_in;
        
        if(debug)
            std::cout << "\t Created a resistor with nodes " << nodes[0] << " " << nodes[1] << " and conductance "<< conductance <<std::endl;
    }

    //Matrix Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> is the same as MatrixXd
    void fill_matrices_DC(Matrices<double>& matrices) override {
        //std::cout << "Filling matrices with resistor with conductance " << conductance << " between nodes " << nodes[0] << " and " << nodes[1] << std::endl;
        
        resize_node_matrices(matrices, nodes, 2);

        add_element_cond(matrices.G_matrix, nodes[0] - 1, nodes[0] - 1, conductance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, nodes[0] - 1, -conductance);
        add_element_cond(matrices.G_matrix, nodes[0] - 1, nodes[1] - 1, -conductance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, nodes[1] - 1, conductance);

    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) override {
        //std::cout << "Filling matrices with resistor with conductance " << conductance << " between nodes " << nodes[0] << " and " << nodes[1] << std::endl;
        
        resize_node_matrices(matrices, nodes, 2);
        
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[0] - 1, nodes[0] - 1, conductance);
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[1] - 1, nodes[0] - 1, -conductance);
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[0] - 1, nodes[1] - 1, -conductance);
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[1] - 1, nodes[1] - 1, conductance);
    }

};


struct DCCurrentSource : Component {
    double current;
    //first node is in, second node is out

    DCCurrentSource(int nodes_in [2], double current_in, bool debug=false){
        component_type = ComponentType::DCCurrentSource;
        
        //TO-DO : FIX NODE ORDER OF CURRENT SOURCES, SHOULD BE THE OTHER WAY ROUND! -------------------------------------------------------------
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        current = current_in;
        
        if(debug)
            std::cout << "\t Created a DC current source with nodes "<< nodes[0] << nodes[1] << " and current "<<current_in<<std::endl;
    }

    void fill_matrices_DC(Matrices<double>& matrices) override {
        //is this order correct?
        //std::cout << "Filling matrices with current source with current " << current << " going into node " << nodes[0] << " from node " << nodes[1] << std::endl;

        resize_node_matrices(matrices, nodes, no_nodes);

        add_element_cond(matrices.i_vector, nodes[0] - 1, current);
        add_element_cond(matrices.i_vector, nodes[1] - 1, -current);

        //Resistor* resistor = new Resistor(nodes, 0.00001);
        //resistor->fill_matrices_DC(matrices);
        //delete(resistor);

    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) override {
        //std::cout << "This is a DC current source and won't be included in the AC analysis." << current << std::endl;
    }
};

struct ACCurrentSource : Component {
    std::complex<double> current;
    
    //first node is positive terminal
    ACCurrentSource(int nodes_in [2], std::complex<double> current_in, bool debug=false){
        component_type = ComponentType::ACCurrentSource;
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        current = current_in;

        if(debug)
            std::cout<<"created a current source with nodes "<<nodes[0]<<nodes[1]<<" and current " <<current<<std::endl;
    }

    std::complex<double> get_AC_source_val() override {
        return current;
    }

    void fill_matrices_DC(Matrices<double>& matrices) override {
        //std::cout << "This is an AC current source and won't be included in the DC analysis." << current << std::endl;
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages,  double freq) override {
        //std::cout << "Filling matrices with current source with current " << current << " going into node " << nodes[0] << " from node " << nodes[1] << std::endl;

        resize_node_matrices(matrices, nodes, no_nodes);

        add_element_cond(matrices.i_vector, nodes[0] - 1, current);
        add_element_cond(matrices.i_vector, nodes[1] - 1, -current);
    }
};

struct DependentCurrentSource : Component {
    double current;
    int control_nodes [2];
    double transconductance;
    
    DependentCurrentSource(int nodes_in [2], int control_nodes_in [2], double transconductance_in, bool debug=false){
        component_type = ComponentType::DependentCurrentSource;
        
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];

        control_nodes[0] = control_nodes_in[0];
        control_nodes[1] = control_nodes_in[1];

        transconductance = transconductance_in;

        if(debug){
            std::cout << "\t Created a voltage controlled current with nodes "<< nodes[0] << " " << nodes[1] << " and transconductance " << transconductance <<std::endl;
            std::cout << "\t Control nodes are NC+: " << control_nodes[1] << " NC-: " << control_nodes[0] << std::endl; 
        }
        
    }

    void fill_matrices_DC(Matrices<double>& matrices) override {

        resize_node_matrices(matrices, nodes, no_nodes);
        resize_node_matrices(matrices, control_nodes, no_nodes);

        // add_element_cond(matrices.i_vector, nodes[0] - 1, current);
        // add_element_cond(matrices.i_vector, nodes[1] - 1, -current);

        //nodes[0] is N- (destination for current), nodes[1] is N+, control_nodes[0] is NC-, control_nodes[1] is NC+
        add_element_cond(matrices.G_matrix, nodes[0] - 1, control_nodes[0] - 1, transconductance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, control_nodes[0] - 1, -transconductance);
        add_element_cond(matrices.G_matrix, nodes[0] - 1, control_nodes[1] - 1, -transconductance);
        add_element_cond(matrices.G_matrix, nodes[1] - 1, control_nodes[1] - 1, transconductance);
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) override {
        resize_node_matrices(matrices, nodes, no_nodes);
        resize_node_matrices(matrices, control_nodes, no_nodes);
        
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[0] - 1, control_nodes[0] - 1, transconductance);
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[1] - 1, control_nodes[0] - 1, -transconductance);
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[0] - 1, control_nodes[1] - 1, -transconductance);
        add_element_cond<std::complex<double>>(matrices.G_matrix, nodes[1] - 1, control_nodes[1] - 1, transconductance);
    }
};


struct DCVoltageSource : Component {
    double voltage;
    //first node is positive terminal

    DCVoltageSource(int nodes_in [2], double voltage_in, bool debug=false){
        component_type = ComponentType::DCVoltageSource;
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        voltage = voltage_in;

        if(debug)
            std::cout << "\t Created a DC voltage source with nodes "<< nodes[0] << " " << nodes[1]<<" and voltage "<<voltage<<std::endl;
    }

    void fill_matrices_DC(Matrices<double>& matrices) override {
        //std::cout << "Filling matrices with voltage source with voltage " << voltage << std::endl;
        
        matrices.b_index++;
        resize_node_matrices(matrices, nodes, no_nodes);
        matrices.B_matrix.col(matrices.b_index - 1).setZero();

        matrices.e_vector.conservativeResize(matrices.e_vector.rows()+1, 1);
        matrices.e_vector[matrices.e_index] += voltage;
        matrices.e_index++;
        //at the end we create the c matrix by taking the transpose of the b matrix
        
        add_element_cond(matrices.B_matrix, nodes[0] - 1, matrices.b_index - 1, (double)1.0);
        add_element_cond(matrices.B_matrix, nodes[1] - 1, matrices.b_index - 1, -(double)1.0);

        //shunt resistor
        //Resistor* resistor = new Resistor(nodes, 0.001);
        //resistor->fill_matrices_DC(matrices);
        //delete(resistor);
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) override {
        
        //short circuit
        //std::cout << "Filling matrices with voltage source with voltage " << voltage << std::endl;
        
        matrices.b_index++;
        resize_node_matrices(matrices, nodes, no_nodes);
        matrices.B_matrix.col(matrices.b_index - 1).setZero();

        matrices.e_vector.conservativeResize(matrices.e_vector.rows()+1, 1);
        matrices.e_vector[matrices.e_index] += 0;
        matrices.e_index++;
        //at the end we create the c matrix by taking the transpose of the b matrix
        
        add_element_cond(matrices.B_matrix, nodes[0] - 1, matrices.b_index - 1, (std::complex<double>)1);
        add_element_cond(matrices.B_matrix, nodes[1] - 1, matrices.b_index - 1, (std::complex<double>)-1);
    }
};

struct ACVoltageSource : Component {
    std::complex<double> voltage;
    
    //first node is positive terminal
    ACVoltageSource(int nodes_in [2], std::complex<double> voltage_in, bool debug=false){
        component_type = ComponentType::ACVoltageSource;
        nodes[0] = nodes_in[0];
        nodes[1] = nodes_in[1];
        voltage = voltage_in;

        if(debug)
            std::cout<<"\tCreated an AC voltage source with nodes "<< nodes[0] << " " << nodes[1] << " and voltage "<<voltage<<std::endl;
    }

    std::complex<double> get_AC_source_val() override {
        return voltage;
    }

    void fill_matrices_DC(Matrices<double>& matrices) override {
        
        // short circuit
        
        //std::cout << "Filling matrices with voltage source with voltage " << voltage << std::endl;
        
        matrices.b_index++;
        resize_node_matrices(matrices, nodes, no_nodes);
        matrices.B_matrix.col(matrices.b_index - 1).setZero();

        matrices.e_vector.conservativeResize(matrices.e_vector.rows()+1, 1);
        matrices.e_vector[matrices.e_index] += 0;
        matrices.e_index++;
        //at the end we create the c matrix by taking the transpose of the b matrix
        
        add_element_cond(matrices.B_matrix, nodes[0] - 1, matrices.b_index - 1, 1.0);
        add_element_cond(matrices.B_matrix, nodes[1] - 1, matrices.b_index - 1, -1.0);
    }

    void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) override {
        //std::cout << "Filling matrices with voltage source with voltage " << voltage << std::endl;
        
        matrices.b_index++;
        resize_node_matrices(matrices, nodes, no_nodes);
        matrices.B_matrix.col(matrices.b_index - 1).setZero();

        matrices.e_vector.conservativeResize(matrices.e_vector.rows()+1, 1);
        matrices.e_vector[matrices.e_index] += voltage;
        matrices.e_index++;
        //at the end we create the c matrix by taking the transpose of the b matrix
        
        add_element_cond(matrices.B_matrix, nodes[0] - 1, matrices.b_index - 1, (std::complex<double>)1);
        add_element_cond(matrices.B_matrix, nodes[1] - 1, matrices.b_index - 1, (std::complex<double>)-1);

    }
};

