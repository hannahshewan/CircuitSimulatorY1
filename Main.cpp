#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include "MatrixUtilities.h"
#include "ComponentBasics.h"
#include "LinearComponents.h"
#include "ReactiveComponents.h"
#include "Diode.h"
#include "MOSFETs.h"
#include "BJTs.h"
#include "ExportSpiceNetlist.h"
#include "Analysis.h"
#include "Parser.h"

bool parse_line(const std::string line, std::vector<Component*>& components){
    std::vector<std::string> line_vector = split(line);
        
    if(line_vector[0] == ".end"){
        //std::cout<<"stopping here"<<std::endl;
        return true;
        
    } else if(line_vector[0] == ".ac"){
        //AC_command(std::stod(line_vector[1]), std::stod(line_vector[2]), std::stod(line_vector[3]));
        //std::cout << "Detected .ac command " << std::endl;
        if(line_vector.size() != 5){
            std::cout << "Error - Expected 5 arguments for .ac command." << std::endl;
            //return false;
        } else {
            solve_ac_analysis(components, parse_val(line_vector[2]), parse_val(line_vector[3]), parse_val(line_vector[4]));
            //return false;
        }

        //std::cout << " ac command finished " << std::endl;

        return false;

    } else if(line_vector[0] == ".op"){
        Eigen::VectorXd dc_sols;
        if(solve_DC_op(components, dc_sols) == true){
            
            //ugly, but works: Loop through components to find number of voltage sources
            int no_volt_srcs = 0;
            for(int c = 0; c < components.size(); c++){
                if(components[c]->component_type == ComponentType::DCVoltageSource){
                    no_volt_srcs++;
                }
            }
            
            std::cout << "DC Operating point solutions for given circuit: " << std::endl;
            
            for(int i = 0; i < dc_sols.rows() - no_volt_srcs; i++){
                std::cout << "Node " << i+1 << ":\t\t" << dc_sols(i) << " voltage" << std::endl;
            }
            
            for(int i = dc_sols.rows() - no_volt_srcs; i < dc_sols.rows(); i++){
                std::cout << "Voltage Src " << i - dc_sols.rows() + no_volt_srcs + 1 << ":\t" << dc_sols(i) << " device_current" <<std::endl;
            }
        }



    }else if (line_vector[0][0] != '.' && line_vector[0][0] != '*'){
        add_component(line,line_vector, components);
            
    } else if(line_vector[0][0] != '*'){
        std::cout << "Skipping line that is neither a comment or valid command/component: " << line << std::endl;
    }

    return false;
}


int execute_netlist(std::string netlist_path){
    std::ifstream infile;
    std::vector<Component*> components;

    //std::cout<<"insert file address"<<std::endl;
    //std::cin>>netlist;
    
    infile.open(netlist_path);

    if(!infile.is_open()){
        std::cout<<" Error opening specified netlist"<<std::endl;
        return EXIT_FAILURE;
    }
    else{
        std::string line;
        while(getline(infile, line)){
            bool finished = parse_line(line, components);
            if(finished == true){
                return 0;
            }
        }
    }

    for(int c = 0; c < components.size(); c++){
        delete components[c];
    }

    return 0;
}



int main(int argc, char *argv[]){
    std::cout << "------------------------- NEW PROGRAM TEST RUN STARTING HERE -----------------------" << std::endl;

    if(argc > 1){ //parsed netlist to .exe. Simply execute this
        std::cout << "Netlist parsed to .exe. Attempting to parse... " << std::endl;
        std::string file_argument = argv[1];
        
        if(file_argument.size() < 5){
            std::cout << "Specified file name not long enough to be valid file path." << std::endl;
            return EXIT_FAILURE;
        }

        std::string file_extension = file_argument.std::string::substr(file_argument.size()-4, std::string::npos);

        if(file_extension == ".txt" || file_extension == ".net"){
            std::cout << "Executing netlist" << file_argument << std::endl;
            return execute_netlist(file_argument);
        } else if (file_extension == ".asc"){
            std::cout << " To create a netlist automatically from an .asc file, please specify the path to your LTSpice executable with quotation marks around it:" << std::endl;
            std::string LTspice_exe_path;
            getline(std::cin, LTspice_exe_path);
            if(LTspice_exe_path == ""){ //default case for debugging purposes
                LTspice_exe_path = R"("C:\Program Files\LTC\LTspiceXVII\XVIIx64.exe")";
            }

            std::string netlist_path = Get_spice_netlist(LTspice_exe_path, file_argument);
            return execute_netlist(netlist_path);
        } else {
            std::cout << "Specified file not valid. Valid file types are .txt, .net or .asc" << std::endl;
            return EXIT_FAILURE;
        }


        
    } else { // no arguments parsed, user can parse in netlist dynamically
        
        std::cout << "No netlist parsed to .exe. You can enter your netlist commands directly instead: " << std::endl;
        std::vector<Component*> components;

        std::string user_input;
        getline(std::cin, user_input);
        while(parse_line(user_input, components) == false){
            user_input = "";
            getline(std::cin, user_input);
        }

        for(int c = 0; c < components.size(); c++){
            delete components[c];
        }

    }

    //std::string LTspice_exe_path = R"("C:\Program Files\LTC\LTspiceXVII\XVIIx64.exe")"; 
    //std::string schematic_path = R"(C:\Users\Joachim\Desktop\GitHubRepos\CircuitSimulator\code\TestingCircuit.asc)"; 
    
    //std::string netlist_path = Get_spice_netlist(LTspice_exe_path, schematic_path);

    
}