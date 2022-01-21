#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <complex>


template <typename T>
struct Matrices {

    Eigen::Matrix<T, -1, -1> A;
    //Decomposition of A
    Eigen::Matrix<T, -1, -1> G_matrix;
    Eigen::Matrix<T, -1, -1> B_matrix;
    Eigen::Matrix<T, -1, -1> C_matrix;
    Eigen::Matrix<T, -1, -1> D_matrix;

    //Decomposition of z
    Eigen::Matrix<T, -1, 1> z; //<T, -1, 1> is vector

    Eigen::Matrix<T, -1, 1> i_vector;
    Eigen::Matrix<T, -1, 1> e_vector;

    int e_index; 
    int b_index;

    Matrices(){
        A = Eigen::Matrix<T, -1, -1>::Zero(0,0);
        
        G_matrix = Eigen::Matrix<T, -1, -1>::Zero(0, 0);
        B_matrix = Eigen::Matrix<T, -1, -1>::Zero(0, 0);
        C_matrix = Eigen::Matrix<T, -1, -1>::Zero(0, 0);
        D_matrix = Eigen::Matrix<T, -1, -1>::Zero(0, 0);

        z = Eigen::Matrix<T, -1, 1>::Zero(0);

        i_vector = Eigen::Matrix<T, -1, 1>::Zero(0);
        e_vector = Eigen::Matrix<T, -1, 1>::Zero(0);

        e_index = 0;
        b_index = 0;    
    }
};

template<typename T>
void resize_node_matrices(Matrices<T>& matrices, int nodes [], int no_nodes){
    int highest_node = 0;

    if(no_nodes > 3){
        std::cout << "WARNING! Potential memory leak. More than 3 nodes given to resize node matrices - max no. of terminals is 3. Skipping..." << std::endl;
        return; 
    }

    for(int n = 0; n < no_nodes; n++){
        if(nodes[n] > highest_node){
            highest_node = nodes[n];
        }
    }

    
    
    //int highest_node = nodes[0] > nodes[1] ? nodes[0] : nodes[1];
    //std::cout << "of these two nodes " << nodes[0] << nodes[1] << " then this node is highest: " << highest_node << std::endl;
    
    //equivalent to conductance (G) matrix width or height
    int nodes_size = highest_node;
    int nodes_to_be_added = nodes_size - matrices.G_matrix.rows();

    if (nodes_to_be_added > 0){

        resize_matrix_zero_init(matrices.G_matrix, nodes_size, nodes_size);
        resize_vector_zero_init(matrices.i_vector, nodes_size);
    }

    if(matrices.b_index > matrices.B_matrix.cols() || nodes_to_be_added > 0){
        resize_matrix_zero_init(matrices.B_matrix, nodes_size, matrices.b_index);
    }
}

enum class ComponentType{
    Resistor,
    Capacitor,
    Inductor,
    DCCurrentSource,
    DCVoltageSource,
    ACCurrentSource,
    ACVoltageSource,
    DependentCurrentSource,
    Diode,
    NPN_BJT, //differentiate between npn & pnp maybe?
    PNP_BJT,
    NMOS,
    PMOS
};


struct Component {
    int nodes [3]; //if working with a two-terminal device, only nodes 0 & 1 are used
    int no_nodes;
    ComponentType component_type;

    //base constructor is executed after derived constructor
    Component(){
        //std::cout << " component constructor called " << std::endl;
        if (component_type == ComponentType::NPN_BJT || component_type == ComponentType::PNP_BJT || component_type == ComponentType::NMOS || component_type == ComponentType::PMOS){
            no_nodes = 3;
        } else {
            no_nodes = 2;
        }
    }

    bool is_nonlinear(){
        if (component_type == ComponentType::Diode || component_type == ComponentType::NPN_BJT || component_type == ComponentType::PNP_BJT || 
            component_type == ComponentType::NMOS || component_type == ComponentType::PMOS){
            return true;
        } else {
            return false;
        }
    }


    virtual std::complex<double> get_AC_source_val(){
        std::cout << "Get AC Source Val called on component base struct. Error" << std::endl;
        return 0;
    }

    virtual void fill_matrices_DC(Matrices<double>& matrices) {
        std::cout << "Fill matrices DC on component base struct called. Error" << std::endl;
    }

    virtual void fill_matrices_DC(Matrices<double>& matrices, const Eigen::VectorXd& volt_guesses) {
        std::cout << "Fill matrices DC on component base struct called. Error" << std::endl;
    }

    virtual void fill_matrices_AC(Matrices<std::complex<double>>& matrices, const Eigen::VectorXd& nodal_voltages, double freq) {
        std::cout << "Fill matrices AC on component base struct called. Error" << std::endl;
    }
};


struct AC_command{
    int starting_frequency;
    int stopping_frequency;
    int points_per_decade;

    AC_command(int starting, int stopping, int rate){
        starting_frequency = starting;
        stopping_frequency = stopping;
        points_per_decade = rate;

        std::cout<<"starting at "<< starting_frequency << " stopping at "<< stopping_frequency << " points per decade is "<< points_per_decade << std::endl;

    }

};





