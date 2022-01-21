#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>


void print_matrix_eq(const Matrices<double> matrices){
    //print_matrix(matrices.G_matrix, "G");
    std::cout << "------ PRINTING MATRICES RELEVANT FOR SOLVE ------" << std::endl;
    print_matrix<double>(matrices.A, "A");
    print_matrix<double>(matrices.z, "Z");

    //print_matrix(matrices.G_matrix, "G");
    //print_matrix(matrices.i_vector, "i");
    //print_matrix(matrices.e_vector, "e");
}

void print_matrix_eq(const Matrices<std::complex<double>> matrices){
    //print_matrix(matrices.G_matrix, "G");
    std::cout << "------ PRINTING MATRICES RELEVANT FOR SOLVE ------" << std::endl;
    print_matrix<std::complex<double>>(matrices.A, "A");
    print_matrix<std::complex<double>>(matrices.z, "Z");

    //print_matrix(matrices.G_matrix, "G");
    //print_matrix(matrices.i_vector, "i");
    //print_matrix(matrices.e_vector, "e");
}





template <typename T>
void compose_matrices(Matrices<T>& matrices){
    matrices.C_matrix = matrices.B_matrix.transpose();

    int no_nodes = matrices.G_matrix.rows();
    int no_volt_sources = matrices.B_matrix.cols();

    int k_size = no_nodes + no_volt_sources;

    //Combine matrices into, A, x and z
    // see https://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
    //K is A before removing ground node
    //"Conductance" matrix
    Eigen::Matrix<T, -1, -1> K = Eigen::Matrix<T, -1, -1>::Zero(k_size, k_size);
    
    K.block(0, 0, no_nodes, no_nodes) = matrices.G_matrix;

    if(matrices.b_index > 0){
        K.block(0, no_nodes, no_nodes, no_volt_sources) = matrices.B_matrix;
        K.block(no_nodes, 0, no_volt_sources, no_nodes) = matrices.C_matrix;
    }

    //D matrix not initialized yet since we aren't dealing with dependent voltage sources
    //if(no_volt_sources > 0){
    //    K.block(no_nodes, no_nodes, no_volt_sources, no_volt_sources) = matrices.D_matrix;
    //}
     
    matrices.A = K;

    matrices.z = Eigen::Matrix<T, -1, 1>::Zero(k_size);
    matrices.z.head(no_nodes) = matrices.i_vector.tail(no_nodes);
    matrices.z.tail(no_volt_sources) = matrices.e_vector;
}

template <typename T>
Eigen::Matrix<T, -1, 1> solve_matrix_eq(Matrices<T>& matrices){
    
    compose_matrices(matrices);
    
    // std::cout << "matrices right before solve" << std::endl;
    // print_matrix_eq(matrices);
    
    Eigen::Matrix<T, -1, 1> x = matrices.A.inverse() * matrices.z;

    //std::cout << "x: " << std::endl << x << std::endl;
    return x;
}


bool iterations_converged(const Eigen::VectorXd& cur_nodal_guesses, const Eigen::VectorXd& prev_nodal_guesses){

    double rel_tol = 0.0001;
    double abs_tol = 0.000001;
    //double rel_tol = 0.0000001;
    //double abs_tol = 0.000001;

    for(int i = 0; i < cur_nodal_guesses.rows(); i++){
        if(abs(cur_nodal_guesses(i) - prev_nodal_guesses(i)) > rel_tol*cur_nodal_guesses(i) + abs_tol){
            
            //std::cout << "This nodal guess " << cur_nodal_guesses(i) << " is not close enough to " << prev_nodal_guesses(i) << std::endl;
            return false;
        }
    }
    return true;
}



// general flow for non linear DC OP
// 1. populate base matrix with linear components already in circuit. done^^
// 2. guess all nodal voltages
// 3. Create equivalent linearized circuits for non-linear components based on guessed nodal voltages
// 4. populate matrix with the new linearized circuits
// 5. solve for voltages - these voltages are our next guess
// 6. repeat from step 3. until max condition met

bool solve_DC_op(const std::vector<Component*>& components, Eigen::VectorXd& sols){
    
    Matrices<double> linear_matrices;
    bool linear_only = true;

    //We add all the linear components to the matrices.

    //std::cout << "Adding linear components to matrices..." << std::endl;

    for (int c = 0; c < components.size(); c++){
        if(components[c]->is_nonlinear() == false){
            components[c]->fill_matrices_DC(linear_matrices);
        } else {
            
            linear_only = false;
            //Since we don't know the linearized models yet, only expand the matrices for non-linear components
            //std::cout << "Resizing matrices with nodes from non-linear component with nodes " << components[c]->nodes[0] << " and " << components[c]->nodes[1] << std::endl;
            
            resize_node_matrices(linear_matrices, components[c]->nodes, components[c]->no_nodes);
            
        }
    }

    if(linear_only == true){
        std::cout << " No non-linear components detected. Simple DC OP solve will be performed..." << std::endl;
        sols = solve_matrix_eq(linear_matrices);
        return true;
    } else {
        
        compose_matrices(linear_matrices);
        //print_matrix_eq(linear_matrices);


        //to help out with convergence, we add a large leakage resistor from each node to ground
        
        for(int node = 0; node < linear_matrices.G_matrix.rows(); node++){
            int nodes [2] = {node, 0};
            Resistor* resistor = new Resistor(nodes, 0.00000001);
            resistor->fill_matrices_DC(linear_matrices);
            delete(resistor);
        }


        Matrices<double> guess_matrices = linear_matrices; 
        
        //Initially we guess all unknowns to be ones
        //Eigen::VectorXd prev_nodal_guesses = Eigen::VectorXd::Ones(guess_matrices.z.rows());
        Eigen::VectorXd prev_nodal_guesses = Eigen::VectorXd::Ones(guess_matrices.z.rows());
        //prev_nodal_guesses << 3, 2.97, 2, 1.3, -0.1, -0.2;

        Eigen::VectorXd cur_nodal_guesses = prev_nodal_guesses;

        int max_iterations = 150;

        std::ofstream output_file;
        output_file.open("output.csv");

        if(!output_file.is_open()){
            std::cout << "Error, couldn't create CSV output file. Exiting..." << std::endl;
            return false;
        }


        for(int n = 0; n < max_iterations; n++){
            //std::cout << " I am on iteration " << n << " of the newton-raphson process" << std::endl;
 

            // reset the matrix eq. to only contain linear components
            guess_matrices = linear_matrices; 
            
            //might be better to add non-linear components to their vector and just loop over them without having to check for non-linearity
            
            //use the nodal guesses to fill the matrices with the linearized versions of every non-linear component
            
            for(int c = 0; c < components.size(); c++){
                if(components[c]->is_nonlinear() == true){
                    components[c]->fill_matrices_DC(guess_matrices, cur_nodal_guesses);
                }
            }
            
            cur_nodal_guesses = solve_matrix_eq(guess_matrices);
            
            for(int n = 0; n < cur_nodal_guesses.rows(); n++){
                output_file << cur_nodal_guesses(n) << ",";
            }
            output_file << "\n";
            
            if(iterations_converged(cur_nodal_guesses, prev_nodal_guesses) == true){
                sols = cur_nodal_guesses;
                std::cout << "Succesful DC operating point performed after " << n << " iterations." << std::endl; 
                return true;
            }
            

            prev_nodal_guesses = cur_nodal_guesses;

            //print_matrix(cur_nodal_guesses, "nodal guesses");
        }

        std::cout << "Error - after " << max_iterations << " iterations DC operating point could not converge." << std::endl;
        return false;
    }

}


void solve_ac_analysis(std::vector<Component*> components, double points_per_dec, double start_frequency, double stop_frequency){   
    
    Eigen::VectorXd dc_sols;
    bool dc_op_successful = solve_DC_op(components, dc_sols);

    if(dc_op_successful == false){
        return;
    }

    std::cout << "Performing AC analysis" << std::endl;
    

    int no_steps = log10(stop_frequency/start_frequency) * points_per_dec;

    //std::cout << "Performing AC analysis with " << no_steps << " steps total" << std::endl;
    //std::cout<< " Starting at "<< start_frequency << " stopping at "<< stop_frequency << " and points per decade is "<< points_per_dec << std::endl;

    if(no_steps <= 0){
        std::cout << "Negative or zero amount of frequency steps specified. Cancelling AC analysis..." << std::endl;
        return;
    }
    
    std::ofstream output_file;
    output_file.open("output.csv");

    if(!output_file.is_open()){
        std::cout << "Error, couldn't create CSV output file. Exiting..." << std::endl;
        return;
    }

    Matrices<std::complex<double>> base_matrices;

    for (int c = 0; c < components.size(); c++){ //only add non-reactive components once in the beginning
        ComponentType comp_type = components[c]->component_type;
        if(comp_type != ComponentType::Capacitor && comp_type != ComponentType::Inductor){
            components[c]->fill_matrices_AC(base_matrices, dc_sols, 0);
        } else {
            resize_node_matrices(base_matrices, components[c]->nodes, components[c]->no_nodes);
        }
    }

    


    std::complex<double> input_source_phasor = 0;

    //find first AC source to be used for the transfer function
    for (int c = 0; c < components.size(); c++){
        ComponentType comp_type = components[c]->component_type;
        
        if(comp_type == ComponentType::ACVoltageSource || comp_type == ComponentType::ACVoltageSource){
            input_source_phasor = components[c]->get_AC_source_val();
        }
    }

    if(input_source_phasor == (std::complex<double>)0){
        std::cout << "Error - no valid AC input source identified for AC analysis. Cancelling operation..." << std::endl;
        return;
    }

    int chosen_node = 1;

    std::string node_string;
    std::cout << "Please select an output node for the transfer function: " << std::endl;

    getline(std::cin, node_string);

    chosen_node = convert_node_to_int(node_string);

    if(chosen_node > base_matrices.G_matrix.rows()){
        std::cout << "Error - chosen node is not in the netlist." << std::endl;
    }

    for(int n = 0; n <= no_steps; n++){
        
        Matrices<std::complex<double>> matrices = base_matrices;
        
        double freq = pow(10, n/points_per_dec) * start_frequency;

        double ang_freq = 2 * PI * freq;

        for (int c = 0; c < components.size(); c++){
            ComponentType comp_type = components[c]->component_type;
            
            if(comp_type == ComponentType::Capacitor || comp_type == ComponentType::Inductor){
                components[c]->fill_matrices_AC(matrices, dc_sols, ang_freq);
            }
        }

        //compose_matrices(matrices);

        auto sols = solve_matrix_eq<std::complex<double>>(matrices);
        
        //print_matrix_eq(matrices);
        
        std::complex<double> node_phasor = sols(chosen_node - 1)/input_source_phasor;
        
        double magnitude = abs(node_phasor);
        double phase = degrees(arg(node_phasor));

        //std::cout << std::endl << ang_freq << "\t" << magnitude << "\t" << phase << "\t" << node_phasor << std::endl;
        
        output_file << freq << ',' << magnitude << ',' << phase << '\n';
    }
    std::cout << "Succesfully performed AC analysis. Output has been written to output.csv" << std::endl;
    output_file.close();
}

