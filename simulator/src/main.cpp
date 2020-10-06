//  created:    2012/04/18
//  filename:   main.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 6599
//
//  version:    $Id: $
//
//  purpose:    main simulateur
//
//
/*********************************************************************/

#include <tclap/CmdLine.h>
#include <Simulator.h>
#include <X4.h>
#include <X8.h>
#include <SimuImu.h>
#ifdef GL
#include <Parser.h>
#include <Man.h>
#include <SimuUsGL.h>
#endif


using namespace TCLAP;
using namespace std;
using namespace flair::simulator;
using namespace flair::sensor;

int port;
int opti_time;
string xml_file;
string media_path;
string scene_file;
string name;
string address;
string type;

void parseOptions(int argc, char** argv)
{
	try
	{
        CmdLine cmd("Command description message", ' ', "0.1");

        ValueArg<string> nameArg("n","name","uav name, also used for vrpn",true,"x4","string");
        cmd.add( nameArg );

        ValueArg<string> xmlArg("x","xml","xml file",true,"./reglages.xml","string");
        cmd.add( xmlArg );

        ValueArg<int> portArg("p","port","ground station port",true,9002,"int");
        cmd.add( portArg );

        ValueArg<int> optiArg("o","opti","optitrack time ms",false,0,"int");
        cmd.add( optiArg );
#ifdef GL
        ValueArg<string> mediaArg("m","media","path to media files",true,"./","string");
        cmd.add( mediaArg );

        ValueArg<string> sceneArg("s","scene","path to scene file",true,"./voliere.xml","string");
        cmd.add( sceneArg );
#endif
        ValueArg<string> addressArg("a","address","ground station address",true,"127.0.0.1","string");
        cmd.add( addressArg );
        
        ValueArg<string> typeArg("t", "type", "uav type, x4 or x8", true, "x4", "string");
        cmd.add(typeArg);

        cmd.parse( argc, argv );

        // Get the value parsed by each arg.
        port=portArg.getValue();
        address=addressArg.getValue();
        xml_file = xmlArg.getValue();
        opti_time = optiArg.getValue();
        name=nameArg.getValue();
        type = typeArg.getValue();
#ifdef GL
        media_path=mediaArg.getValue();
        scene_file=sceneArg.getValue();
#endif


	} catch (ArgException &e) {
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
        exit(EXIT_FAILURE);
    }
}


int main(int argc, char* argv[]) {
  Simulator *simu;

  parseOptions(argc,argv);

  simu= new Simulator("simulator",opti_time,90);
  simu->SetupConnection(address,port);
  simu->SetupUserInterface(xml_file);
  #ifdef GL
  Parser *gui=new Parser(1024,768,1024,768,media_path,scene_file);
  #endif
  for(int i=0; i<3;i++) {
    stringstream uavName;
    Model *drone;
    uavName << name.c_str() << "_" << i;
    if(type == "x4") {
      drone = new X4(uavName.str(),i);
    } else {
      drone = new X8(uavName.str(),i);
    }
    
#ifdef GL
    SimuUsGL *us_gl=new SimuUsGL(drone,"us",i,0);
#endif
    SimuImu *imu=new SimuImu(drone,"imu",i,0);
  }

  simu->RunSimu();

  delete simu;

	return 0;
}

