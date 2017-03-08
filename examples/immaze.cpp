#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/PathGeometric.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ramp/RASP.h"
#include "ramp/NaiveRASP.h"

namespace po = boost::program_options;
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool yes(const ob::State *state)
{
    return true;
}

ob::ScopedState<ob::RealVectorStateSpace>
make_state(const ob::StateSpacePtr space, double x, double y)
{
    ob::ScopedState<ob::RealVectorStateSpace> state(space);
    double *vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
    vals[0] = x;
    vals[1] = y;
    return state;
}

void draw_path(cv::Mat img_path, std::vector<ob::State*> states)
{
    int n = states.size();
    int r = img_path.rows;
    int c = img_path.cols;
    for (int i = 0; i < n-1; i++)
    {
        double *curr_vals = states[i]->as<ob::RealVectorStateSpace::StateType>()->values;
        double *next_vals = states[i+1]->as<ob::RealVectorStateSpace::StateType>()->values;

        cv::Point p((int) (curr_vals[0] * c), (int) (curr_vals[1] * r));
        cv::Point p_next((int) (next_vals[0] * c), (int) (next_vals[1] * r));
        cv::line(img_path, p, p_next, cv::Scalar(255, 0, 0), 2);
    }
}

void display_path(std::string ofname, cv::Mat &img)
{
    if (ofname.empty())
    {
        cv::imshow("Path", img);
        cv::waitKey(0);
        return;
    }
    cv::imwrite(ofname, img);
}

void write_path(std::string ofname, std::vector<ob::State*> states)
{
    if (ofname.empty())
        return;

    std::ofstream ofs;
    ofs.open(ofname);
    for (size_t i = 0; i < states.size(); ++i)
    {
        double *vals = states[i]->as<ob::RealVectorStateSpace::StateType>()->values;
        for (size_t j = 0; j < 2; ++j)
        {
            ofs << vals[j] << " ";
        }
        ofs << std::endl;
    }
    ofs.close();
}

int main(int argc, char *argv[])
{
    std::string graph_file, map_file;
    double startx, starty, goalx, goaly;
    std::string image_out;
    bool ignore_risk;
    bool precompute;
    std::string path_out;
    bool nosave;

    po::options_description desc("2D Image Planner Options");
    desc.add_options()
        ("help,h", "produce help message")
        ("graph,g", po::value<std::string>(&graph_file)->default_value("data/riskmaps/immaze_basic_01_uniform_1.graphml"), "graph")
        ("map,m", po::value<std::string>(&map_file)->default_value("data/envs/basic.jpg"), "image")
        ("startx", po::value<double>(&startx)->default_value(0.26), "startx")
        ("starty", po::value<double>(&starty)->default_value(0.34), "starty")
        ("goalx",  po::value<double>(&goalx)->default_value(0.68),  "goalx")
        ("goaly",  po::value<double>(&goaly)->default_value(0.34),  "goaly")
        ("image_out", po::value<std::string>(&image_out)->default_value(""), "image output")
        ("path_out", po::value<std::string>(&path_out)->default_value(""), "path output")
        ("ignore_risk,r", po::value<bool>(&ignore_risk)->default_value(false),
             "ignore risk (only considered by incremental RASP planner)")
        ("precompute", po::value<bool>(&precompute)->default_value(false),
             "use precomputation instead")
        ("nosave", po::value<bool>(&nosave)->default_value(false),
             "generate no output (overrides image_out/path_out)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    cv::Mat img_gray = cv::imread(map_file, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img_path = cv::imread(map_file, CV_LOAD_IMAGE_COLOR);

    // Define the state space: R^2
    boost::shared_ptr<ob::RealVectorStateSpace> space(new ob::RealVectorStateSpace(2));
    space->setBounds(0, 1);
    space->setLongestValidSegmentFraction(0.01 / space->getMaximumExtent());
    space->setup();

    // Define the space information
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(&yes);
    si->setup();

    // Define the problem
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(
        make_state(space, startx, starty),
        make_state(space, goalx, goaly));

    ob::PlannerStatus solved;
    if (precompute)
    {
        RAMP::NaiveRASP nrasp_planner(si);
        nrasp_planner.setRoadmap(graph_file);
        nrasp_planner.setProblemDefinition(pdef);
        nrasp_planner.setup();
        solved = nrasp_planner.solve(1.0);
    }
    else
    {
        RAMP::RASP rasp_planner(si);
        rasp_planner.setRoadmap(graph_file);
        rasp_planner.setProblemDefinition(pdef);
        rasp_planner.setIgnoreRisk(ignore_risk);
        rasp_planner.setup();
        solved = rasp_planner.solve(1.0);
    }

    if (!solved)
        return 1;

    if (nosave)
        return 0;

    boost::shared_ptr<og::PathGeometric> path = boost::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
    draw_path(img_path, path->getStates());
    display_path(image_out, img_path);
    write_path(path_out, path->getStates());

    return 0;
}
