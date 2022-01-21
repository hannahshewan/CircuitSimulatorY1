#include <iostream>
#include <Eigen/Dense>

constexpr double PI  = 3.141592653589793238463;

template <typename T>
void resize_matrix_zero_init(Eigen::Matrix<T, -1, -1>& matrix, int new_rows, int new_cols){    
    //only add rows/cols if the new number of rows/cols is actually higher than the current size
    new_rows = new_rows > matrix.rows() ? new_rows : matrix.rows();
    new_cols = new_cols > matrix.cols() ? new_cols : matrix.cols();

    int rows_to_add = new_rows - matrix.rows();
    int cols_to_add = new_cols - matrix.cols();

    matrix.conservativeResize(new_rows, new_cols);

    for (int r = 1; r <= rows_to_add; r++){
        matrix.row(matrix.rows() - r).setZero();
    }

    for (int c = 1; c <= cols_to_add; c++){
        matrix.col(matrix.cols() - c).setZero();
    }
}

template <typename T>
void resize_vector_zero_init(Eigen::Matrix<T, -1, 1>& vector, int new_rows){
    int rows_to_add = new_rows - vector.rows();

    vector.conservativeResize(new_rows);

    for (int r = 1; r <= rows_to_add; r++){
        vector.row(vector.rows() - r).setZero();
    }
}

//set element conditional on the element accessed is valid
template <typename T>
void add_element_cond(Eigen::Matrix<T, -1, -1>& matrix, int row, int col, T e){
    if(row >= 0 && col >= 0){
        matrix(row, col) += e;
    }
}

template <typename T>
void add_element_cond(Eigen::Matrix<T, -1, 1>& vector, int row, T e){
    if(row >= 0){
        vector(row) += e;
    }
}

template <typename T>
double get_element_cond(const Eigen::Matrix<T, -1, -1>& matrix, int row, int col){
    
    if(row >= matrix.rows() || col >= matrix.cols()){ //this is entirely for debug purposes
        std::cout << "Assertion fail incoming! You tried to access node " << row << ',' << col << " of a matrix with size " << matrix.rows() << "," << matrix.cols() << std::endl; 
    }
    
    if(row >= 0 && col >= 0){
        return matrix(row, col);
    } else {
       return 0; 
    }
}

template <typename T>
double get_element_cond(const Eigen::Matrix<T, -1, 1>& vector, int row){
    
    if(row >= vector.rows()){ //this is entirely for debug purposes
        std::cout << "Assertion fail incoming! You tried to access node " << row << " of a matrix with size " << vector.rows() << std::endl; 
    }
    
    if(row >= 0){
        return vector(row);
    } else {
       return 0;
    }
}

template <typename T>
void print_matrix(const Eigen::Matrix<T, -1, -1>& matrix, std::string name = ""){
    std::cout << name << " matrix" << std::endl << matrix << std::endl;
    std::cout << " ------ " << std::endl;
}

template <typename T>
void print_matrix(const Eigen::Matrix<T, -1, 1>& matrix, std::string name = ""){
    std::cout << name << " matrix" << std::endl << matrix << std::endl;
    std::cout << " ------ " << std::endl;
}

double degrees(double radians)
{
    return(radians * (180 / PI));
}

void to_cartesian(double magnitude, double phase, double& x, double& y){
    x = cos(phase) * magnitude;
    y = sin(phase) * magnitude;
}


int convert_node_to_int(std::string node){
    if(node[0] != 'n' && node[0] != 'N' && node[0] != 'p' && node[0] != 'P'){
        return std::stoi(node);
    } else {
        //std::cout << node << std::endl;
        return std::stoi(node.substr(1, std::string::npos));
    }
}
