//wall_follow_path_gen
#include <ros/ros.h> 
#include <visualization_msgs/Marker.h> // need this for publishing markers
#include <geometry_msgs/Point.h> //data type used for markers
#include <string.h>
#include <stdio.h>  
#include <example_rviz_marker/SimpleFloatSrvMsg.h> //a custom message type defined in this package
#include <sensor_msgs/LaserScan.h>

double angle_min_ = 0.0;
double angle_max_ = 0.0;
double angle_increment_ = 0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
int ping_index_ = -1;
std::vector<double> ranges;
std::vector<double> angles;

using namespace std;

//helper fnc for displaying markers in rviz

void init_marker_vals(visualization_msgs::Marker &marker) {
    marker.header.frame_id = "/world"; // reference frame for marker coords
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    // use SPHERE_LIST for a group of markers
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y,   				    
    //        marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below

    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
}


//purpose of this fnc is, given a robot's pose (robot_x,robot_y,robot_heading)
// and a laser scan, (ranges[],angles[]), convert the pings
// to Cartesian coords w/rt world frame: (xvals[],yvals[])
//note: ranges[],angles[] may already have removed out-of-range values,
// so angular increments are not necessarily regular

void polar_to_cartesian(std::vector<double> ranges,
        std::vector<double> angles,
        double robot_x, double robot_y, double robot_heading,
        std::vector<double> &xvals, std::vector<double> &yvals) {
    //converts laser-scan polar coords to x-y Cartesian values, in world frame
    ROS_INFO("converting laser scan to Cartesian points");
    double c_psi = cos(robot_heading);
    double s_psi = sin(robot_heading);
    xvals.clear();
    yvals.clear();
    int npts = ranges.size();
    xvals.resize(npts);
    yvals.resize(npts);
    double r, theta;

    //combines conversion polar->Cartesian, and transform from robot frame to world frame
    for (int i = 0; i < npts; i++) {
        theta = angles[i];
        r = ranges[i];
        xvals[i] = robot_x + c_psi * r * cos(theta) - s_psi * r * sin(theta);
        yvals[i] = robot_y + s_psi * r * cos(theta) + c_psi * r * sin(theta);
    }
}

//returns Euclidean distance from (x1,y1) to (x2,y2)

double euc_dist(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = sqrt(dx * dx + dy * dy);
    return dist;
}


//function to evaluate if a given circle, described by (x_center,y_center) and radius,
//contains any of the points in (xvals,yvals).  Return "true" of contains none of these
//points, else return "false" if found at least one point inside the circle;
//In that case, set "i_conflict" to the index of that point,
// (xvals[i_conflict],yvals[i_conflict])

bool circle_contains_no_pings(double x_center, double y_center, double radius,
        std::vector<double> &xvals, std::vector<double> &yvals, int &i_conflict) {

    bool is_empty = true;
    double rad_tol = radius - 0.001; // use a slightly smaller radius for test;
    // do expect circle is tangent to 2 points
    //brute force: check every ping
    int npts = xvals.size();
    for (int i = 0; i < npts; i++) {
        if (euc_dist(x_center, y_center, xvals[i], yvals[i]) < rad_tol) {
            i_conflict = i;
            return false;
        }
        return true;
    }
}


    //given points (x1,y1) and (x2,y2), find, if possible, the center of a circle of
    //radius "radius", (x_center,y_center), such that the circle passes through both points
    //if the points are too far apart, (greater than 2r), return "false"
    //Else, there are 2 solns.  Choose the soln that is in the same half-space
    // as (Ox,Oy).  That is, a robot pinging points p1 and p2 would be in one half-space
    // of the dividing line passing through p1, p2.  This would be "outside" the sensed
    // obstacle (side closer to the robot with sensor)
    // Assume p1, p2 are in order of laser scan, CCW.  

    bool compute_circle_tangent_to_points(double x1, double y1, double x2, double y2,
            double radius, double Ox, double Oy, double &x_center, double &y_center) {

        //do the geometry: p1->p2 is a line segment; bisect this.  find isoceles triangle with
        // sides r, r and base p1->p2.  Apex of this triangle is the center of the desired circle

        double b = euc_dist(x1, y1, x2, y2);
        if (b > 2 * radius) {
            return false;
        }
        double theta = acos(0.5 * b / radius); // width of base = distance from p1 to p2
        //tframe is a coord frame defined w/ origin at p1 and x-axis from p1 to p2,
        // and z axis parallel to z_world
        //compute center of circle (apex of triangle) in tframe:
        double c_x_wrt_tframe = radius * cos(theta);
        double c_y_wrt_tframe = radius * sin(theta);
        //compute x-axis of tframe w/rt world frame:
        double nx = (x2 - x1) / b;
        double ny = (y2 - y1) / b;
        //hard-coded coord transform to express circle center in world coords:
        x_center = x1 + nx * c_x_wrt_tframe - ny*c_y_wrt_tframe;
        y_center = y1 + ny * c_x_wrt_tframe + nx*c_y_wrt_tframe;
        return true;
    }

    //receive laserscan; create separate vectors ranges and angles, populated only
    // with components for which ping is within range
    void laserCallback(const sensor_msgs::LaserScan & laser_scan) {
        if (ping_index_ < 0) {
            //for first message received, set up the desired index of LIDAR range to eval
            angle_min_ = laser_scan.angle_min;
            angle_max_ = laser_scan.angle_max;
            angle_increment_ = laser_scan.angle_increment;
            range_min_ = laser_scan.range_min;
            range_max_ = laser_scan.range_max;
            // what is the index of the ping that is straight ahead?
            // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
            // but this will do for simple illustration
            ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
            //ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);

        }
        int npings = laser_scan.ranges.size();
        ranges.clear();
        angles.clear();
        double range, angle;
        double range_test = range_max_ - 0.001;
        angle = angle_min_;
        for (int i = 0; i < npings; i++) {
            range = laser_scan.ranges[i];
            if (range < range_test) {
                ranges.push_back(range);
                angles.push_back(angle);
            }
            angle += angle_increment_;
        }
    }

        int main(int argc, char **argv) {
            ros::init(argc, argv, "example_rviz_marker");
            ros::NodeHandle nh;
            ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("example_marker_topic", 0);
            visualization_msgs::Marker marker; // instantiate a marker object
            geometry_msgs::Point point; // points will be used to specify where the markers go

            //set up a service to compute marker locations on request
            //ros::ServiceServer service = nh.advertiseService("rviz_marker_svc", displaySvcCB);

            init_marker_vals(marker);

            //need to receive lidar scans:
            ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
            
            //process:
            // 1) get robot pose w/rt world
            // 2) get a laser scan
            // 3) void polar_to_cartesian(std::vector<double> ranges,
            //        std::vector<double> angles,
            //        double robot_x, double robot_y, double robot_heading,
            //        std::vector<double> &xvals, std::vector<double> &yvals)
            // 4) compute_wall_follow_points(...)

            ros::spin(); //this is essentially a "while(1)" statement, except it
            // forces refreshing wakeups upon new data arrival
            // main program essentially hangs here, but it must stay alive to keep the callback function alive
            return 0; // should never get here, unless roscore dies
        }

        
