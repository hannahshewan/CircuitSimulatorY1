#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

// TODO: Determine if string literals can somehow be accessed without doing this
using namespace std::string_literals;

std::string get_raw_string(std::string const& in)
{
   std::string ret = "";

   for (int c = 0; c < in.size(); c++){
       if(in[c] != ' '){
           ret.push_back(in[c]);
       }        
   }
   
   return ret;
}

std::string Get_spice_netlist(std::string LTspice_exe_path, std::string schematic_path){
    // Given the path to the local LTspice exe file and an LTspice schematic, generate a 
    // cleaned netlist. Returns path to cleaned netlist if successful, empty string if not
    
    std::string netlist_arg = " -netlist ";
    std::string command = LTspice_exe_path.c_str() + netlist_arg + schematic_path.c_str();
    std::cout << "Creating netlist with command " << command.c_str() << std::endl;
    
    int result = system(command.c_str());

    if (result != 0){
        std::cout << "Netlist creation not successful, returning empty string" << std::endl;
        return "";
    }
    // netlist has same path as .asc file but with .net suffix
    std::string netlist_path = schematic_path.erase(schematic_path.size()-4) + ".net";
    std::cout << netlist_path << std::endl;

    std::ifstream netlist_file;
    netlist_file.open(netlist_path);

    if (!netlist_file.is_open()){
        std::cout << "Error opening netlist file" << std::endl;
        return "";
    } 
    
    std::string curr_line;
    std::string cleaned_netlist_txt;

    std::string allowed_cmds [3] = {".op", ".ac", ".end"}; 

    while( std::getline(netlist_file, curr_line) ) {
            
        if (curr_line[0] == '.'){
            // if the line is a command, then check if it is one of the one we are processing
            // otherwise remove it
            bool cmd_valid = false;
                    
            for (int c = 0; c < sizeof(allowed_cmds)/sizeof(*allowed_cmds); c++){
                int cmd_size = allowed_cmds[c].size();
                if ( curr_line.substr(0, cmd_size) == allowed_cmds[c]){
                    cmd_valid = true;
                    break;
                }
            }
            if (cmd_valid == false) {
                continue;
            }
        }
                
        cleaned_netlist_txt += curr_line + '\n';
        //std::cout << curr_line << std::endl;
    }

    netlist_file.close();

    std::ofstream new_netlist;
    new_netlist.open(netlist_path);

    if (!new_netlist.is_open()){
        std::cout << "Unable to create new netlist file" << std::endl;
        return "";
    }
    
    new_netlist << cleaned_netlist_txt;
    new_netlist.close();
    std::cout << "Cleaned netlist succesfully created" << std::endl;
    return netlist_path;
}



/*
int main() {
    // example usage
    
    // the LTSpice .exe is usually in program files somewhere
    std::string LTspice_exe_path = R"("C:\Program Files\LTC\LTspiceXVII\XVIIx64.exe")"; 
    std::string schematic_path = R"(C:\Users\Joachim\Desktop\NetlistExportSchematic.asc)"; 
    
    std::string netlist_path = Get_spice_netlist(LTspice_exe_path, schematic_path);

    return 0;
}*/