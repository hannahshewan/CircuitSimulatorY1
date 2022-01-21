// basic parser reads netlist and parses into data structure of their choosing
/* - Notes: 
    - No error handling at the moment i.e. if the input isn’t meaningful 
    - only ignore commented lines, not when the * is in the middle of the line
    - case sensitive at the moment
- To do:
    - Ac input sources
    - better variable names
    - MOSFET + BJT (skipped just because I don’t have the latest structure definitions but this won’t take more than a couple mins to implement)
- to ask:
    - When there’s no unit for parseval */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

//turns lines into a vector of words
std::vector<std::string> split(std::string split_this){
    std::stringstream ss(split_this);
    std::string word_buffer;
    std::vector<std::string> words;
    while (ss >> word_buffer){
        words.push_back(word_buffer);
    }
    return words;
}

void convert_nodes_to_ints(std::vector<std::string> nodes, int start_index, int stop_index, int (&nodes_int)[2]){
    int x = 0;
    for (int i = start_index; i <= stop_index; i++){   
        if(nodes[i]!="0"){
            nodes_int[x]=std::stoi(nodes[i].substr(1, 3));
        }
        x++;
    }      
    return;
}




double parse_val(const std::string& value){
    std::size_t unit_pos = value.find_first_not_of("0123456789.");
    double expanded = std::stod(value);

    if(unit_pos == std::string::npos){
        return expanded;
    }

    char unit = value[unit_pos];

    switch (unit){
        case 'p':
            return expanded*pow(10,-12);  
        case 'n':
            return expanded*pow(10,-9);
        case 'u':
            return expanded*pow(10,-6);
        case 'm':
            return expanded*pow(10,-3);
        case 'k':
            return expanded*pow(10,3);
        case 'M': //checking for 'M' rather than 'Meg'
            return expanded*pow(10,6);
        case 'g':
            return expanded*pow(10,9);
       default:
            std::cout<<"Units of value not recognised for "<< value << std::endl;
            return expanded;
    }
    
}

void add_source(std::string line, std::vector<std::string> line_vector, std::vector<Component*>& components, bool is_current_src){
    
    
    if( line_vector.size() == 4){ // DC sources must have 4 parameters precisely
        if(is_current_src){
            int nodes[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};

            //reverse node order, because apparently it is the other way round.
            int tmp = nodes[0];
            nodes[0] = nodes[1];
            nodes[1] = tmp;

            DCCurrentSource* ct = new DCCurrentSource(nodes, parse_val(line_vector[3]));
            components.push_back(ct);
        } else {
            int nodes[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};
            DCVoltageSource* vt = new DCVoltageSource(nodes, parse_val(line_vector[3]), true);
            components.push_back(vt);
        }
    } else if(line_vector[3][0]=='A' && line_vector[3][1]=='C'){ //AC sources
        
        double amplitude, phase;

        if(line_vector.size() == 5){ //specification AC source
            amplitude = std::stod( line_vector[3].substr( line_vector[3].find_first_of("(") + 1, std::string::npos));
            phase = std::stod( line_vector[4].substr(0, line_vector[4].find_first_of(")")));
        } else if (line_vector.size() == 6) { //LT spice AC source
            amplitude = std::stod( line_vector[4] );
            phase = std::stod( line_vector[5] );
        } else {
            std::cout << "Incorrect number of AC source arguments. Skipping..." << std::endl;
            return;
        }

        double real, imag;
        to_cartesian(amplitude, phase, real, imag);
        std::complex<double> source_val = real + imag * 1i;

        //std::cout << "Adding AC source with amplitude & phase " << amplitude << " " << phase << std::endl;
        
        int nodes[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};

        if(is_current_src){
            ACCurrentSource* accs = new ACCurrentSource(nodes, source_val, true);
            components.push_back(accs);
        } else {
            ACVoltageSource* acvs = new ACVoltageSource(nodes, source_val, true);
            components.push_back(acvs);
        }
    }
}
    
void add_component(std::string line, std::vector<std::string> line_vector, std::vector<Component*>& components){
    switch (line[0]){
        case 'R':
            if (line_vector.size() == 4){
                
                int nodes_resistor[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};
                Resistor* rs = new Resistor(nodes_resistor, 1.0/parse_val(line_vector[3]), true);
                
                components.push_back(rs);
            }
            else{
                std::cout<<"Incorrect number of parameters specified (4 required for resistor): "<< line <<std::endl;
            }
            return;
        case 'C':
            //std::cout<<"adding a capacitor"<<std::endl;
            if (line_vector.size() == 4){
                int nodes_capacitor[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};

                Capacitor* ct = new Capacitor(nodes_capacitor, parse_val(line_vector[3]), true);
                components.push_back(ct);
            }
            else{
                std::cout<<"Incorrect number of parameters specified (4 required for capacitor): " << line << std::endl;
            }
            return;
        case 'L':
            //std::cout<<"adding an inductor"<<std::endl;
            if (line_vector.size() == 4){
                int nodes_inductor[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};
                
                Inductor* lt = new Inductor(nodes_inductor, parse_val(line_vector[3]), true);
                components.push_back(lt);
            }
            else{
                std::cout<<"Incorrect number of parameters specified (4 required for capacitor): " << line << std::endl;
            }
            return;
        case 'D':
            if (line_vector.size() == 4){
                int nodes_diode[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};
                Diode* dt = new Diode(nodes_diode, true);
                components.push_back(dt);
            }
            else{
                std::cout<<"Incorrect number of parameters specified (4 required for diode): " << line << std::endl;
            }
            return; 
            
        case 'Q':
            std::cout<<"adding a BJT"<<std::endl;
            if (line_vector.size() == 5 || line_vector.size() == 6){
                int nodes_BJT [3] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2]), convert_node_to_int(line_vector[3])};

                std::string model_name;
                if(line_vector.size() == 5){
                    model_name = line_vector[4];
                } else {
                    model_name = line_vector[5];
                }

                if(model_name == "NPN" || model_name == "2N2222"){
                    NPN_BJT* bjt = new NPN_BJT(nodes_BJT, true);
                    components.push_back(bjt);
                } else if(model_name == "PNP"){
                    //PMOS* pm = new PMOS(nodes_mosfet, true);
                    //components.push_back(pm);
                } else {
                    std::cout << "Error - model name specified not valid: " << model_name << std::endl;
                    return;
                }
            }
            else{
                std::cout<<"incorrect number of paramters"<<std::endl;
            }
            return;
        case 'M':
            //std::cout<<"and a MOSFET"<<std::endl;
            //LTSpice syntax with bulk node is considered valid as well
            if (line_vector.size() == 5 || line_vector.size() == 6){

                int nodes_mosfet [3] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2]), convert_node_to_int(line_vector[3])};

                std::string model_name;
                if(line_vector.size() == 5){
                    model_name = line_vector[4];
                } else {
                    model_name = line_vector[5];
                }
                if(model_name == "NMOS" || model_name[model_name.size() - 1] == 'N'){
                    NMOS* nm = new NMOS(nodes_mosfet, 1, 1, 400, true);
                    components.push_back(nm);
                } else if(model_name == "PMOS" || model_name[model_name.size() - 1] == 'P'){
                    PMOS* pm = new PMOS(nodes_mosfet, 1, 1, 400, true);
                    components.push_back(pm);
                } else {
                    std::cout << "Error - model name specified not valid: " << model_name << std::endl;
                    return;
                }
                
            } else {
                std::cout<< "Error - incorrect number of parameters for MOSFET. Expected 5 or 6 but recieved " << line_vector.size() << std::endl;
            }
            return;
        case 'G':
            if (line_vector.size() == 6){
                int nodes_vccs[2] = {convert_node_to_int(line_vector[1]) , convert_node_to_int(line_vector[2])};
                int control_nodes[2] = {convert_node_to_int(line_vector[3]) , convert_node_to_int(line_vector[4])};
                DependentCurrentSource* dit = new DependentCurrentSource(nodes_vccs, control_nodes, parse_val(line_vector[5]), true);
                components.push_back(dit);
            }
            else{
                std::cout<<"Incorrect number of parameters specified (6 required for dependent current source): " << line << std::endl;
            }
            return;
        case 'V':
            add_source(line, line_vector, components, false);
            return;
        case 'I':
            add_source(line, line_vector, components, true);
            return;
        default:
            std::cout<<"Error: not a valid component: " << line << std::endl;
            return;
    }
}



/*
int main(){
    std::string netlist;  
    std::ifstream infile;
    std::vector<Component*> components;

    std::cout<<"insert file address"<<std::endl;
    std::cin>>netlist;
    infile.open(netlist);

    if(!infile.is_open()){
        std::cout<<"error opening file"<<std::endl;
        return EXIT_FAILURE;
    }
    else{
        std::string line;
        while(getline(infile, line)){ //does the line ignore whitespace?
            std::vector<std::string> line_vector = split(line);
            if(line[0]!='*'){ 
                if(line == ".end"){
                    std::cout<<"stopping here"<<std::endl;
                    break; //need to stop the program here, not just the if statement
                }
                else{
                    if(line[0]!= '.'){
                      add_component(line,line_vector, components);

                    }
                    else{
                        AC_command(std::stod(line_vector[1]), std::stod(line_vector[2]), std::stod(line_vector[3]));
                    }
                }
            }
            else{
                std::cout<<"skipping this line since it's a comment"<<std::endl;
            }
        }
    }
}*/
