// A common time system
#include <gpstk/CommonTime.hpp>

// Classes for handling RINEX files with meteorological parameters
#include <gpstk/RinexMetData.hpp>
#include <gpstk/RinexMetHeader.hpp>
#include <gpstk/RinexMetStream.hpp>

// This library
#include "Meteorological.h"

using namespace controller;

// Default constructor
Meteorological::Meteorological() : World("meteorological"),
	te(273), pr(1000.0), hu(95.0), rate(1.0), ws(0.0), wd(0.0) {}

// REQUIRED METHODS

// All sensors must be configured using the current model information and the SDF
void Meteorological::Configure(sdf::ElementPtr root, gazebo::physics::WorldPtr world)
{
	// Save the world pointer
	worldPtr = world;

	// GRAB THE DEFAULT PARAMETERs /////////////////////////////////////////////////////

	// Rate at which /world messages are posted
	rate = GetSDFDouble(root,"rate",rate);				// Broadcast rate
	te   = GetSDFDouble(root,"default.kelvin",te);		// Temperarture
	pr   = GetSDFDouble(root,"default.millibars",pr);	// Pressure
	hu   = GetSDFDouble(root,"default.relhumidity",hu);	// Humidity
	ws   = GetSDFDouble(root,"wind.speed",ws);			// Humidity
	wd   = GetSDFDouble(root,"wind.direction",wd);		// Humidity

	// PROCESS ANY RINEX DATA  //////////////////////////////////////////////////////////

	// Will be useful for parsing the sdf
	sdf::ElementPtr el;

	// Extract any meterological data
	try
	{
		el = root->GetElement("rinex")->GetFirstElement();
		do
		{
			// Obtain the URI
			std::string uri;
			el->GetValue()->Get(uri);

			// Open meteorological data file
			gpstk::RinexMetStream rms(
				gazebo::common::SystemPaths::Instance()->FindFileURI(uri).c_str()); 
			gpstk::RinexMetHeader rmh;
			gpstk::RinexMetData   rmd;

		// Let's read the header (may be skipped)
		rms >> rmh;

		// Read data into linked list
		while (rms >> rmd)
		{
		  // We need to specify that this file is in UTC
		  rmd.time.setTimeSystem(gpstk::TimeSystem::GPS);
		  ml.push_back(rmd);
		}

		}
		while(el = el->GetNextElement());

		// Set the iterator to the beginning of the linked list
		mi = ml.begin();
	}
	catch (const std::exception& e)
	{
		ROS_WARN("Could not obtain any meteorological information: %s",e.what());
	}

	// ISSUE A MODULE RESET //////////////////////////////////////////////////////////

	Reset();
}

// All sensors must be resettable
void Meteorological::Reset()
{
	// RESET MODULE STATE  ///////////////////////////////////////////////////////////

	// Default meterological values
	t = te;
	h = hu;
	p = pr;

	// Reset the RINEX iterator
	mi = ml.begin();

	// START PUBLISHING DATA //////////////////////////////////////////////////////////

	// Create and initialize a new Gazebo transport node
	nodePtr = gazebo::transport::NodePtr(new gazebo::transport::Node());
	nodePtr->Init(worldPtr->GetName());

	// Create a publisher on the ~/wind topic
	pubPtr = nodePtr->Advertise<msgs::Meteorological>("~/global/meteorological");

	// CREATE A CALLBACK TIMER ///////////////////////////////////////////////////////

	// ROS timer respects gazebo
	timer = rosNode.createTimer(
		ros::Duration(1.0/rate),    	// Duration
		&Meteorological::Update,   		// Callback
		this
	);     
}

// Send a gazebo meterological message
void Meteorological::Update(const ros::TimerEvent& event)
{
	// Try to see if we can grab some weather data from the RINEX file
	try
	{
		// Seek for the data
		while ((!ml.empty())  && (mi!= ml.end() && (*mi).time < GetTime(gpstk::TimeSystem::UTC))) 
		   mi++; 

		// Set the parent class values, so other children can access them!
		t = (*mi).data[gpstk::RinexMetHeader::TD];
		h = (*mi).data[gpstk::RinexMetHeader::HR];
		p = (*mi).data[gpstk::RinexMetHeader::PR];
	}
	catch(const std::exception& e)
	{
		ROS_WARN("Problem querying weather data: %s", e.what());
	}

	// Assemble the epoch message, with or without RINEX data
	msg.mutable_wind()->set_speed(ws);
	msg.mutable_wind()->set_direction(wd);
	msg.set_temperature(t);
	msg.set_pressure(p);
	msg.set_humidity(h);

	// Publish wind information to all subscribers
	pubPtr->Publish(msg);
}