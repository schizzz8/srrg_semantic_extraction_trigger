#include <iostream>
#include <string>
#include <math.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "srrg_semantic_extraction_trigger/semantic_extraction_trigger.h"

#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_types/cloud_3d.h"


using namespace std;
using namespace srrg_core;
using namespace srrg_semantic_extraction_trigger;

float resolution=0.05f;
Eigen::Vector3f origin;

int main(int argc, char** argv){

    SemanticExtractionTrigger trigger;
    \
    MessageReader reader;
    reader.open(argv[1]);


    Eigen::Matrix3f camera_matrix;
    camera_matrix << 277.127,0,160.5,
            0,277.127,120.5,
            0,0,1;

    Eigen::Isometry3f offset = Eigen::Isometry3f::Identity();
    offset.translate(Eigen::Vector3f(-0.087,0.0475,1.5));
    offset.rotate(Eigen::Quaternionf(0.5,-0.5,0.5,-0.5));


    BaseMessage* msg = 0;
    while ((msg = reader.readMessage())) {
        msg->untaint();
        BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(msg);
        if (sensor_msg) {
            cerr << "[INFO] Reading message of type: " << sensor_msg->tag() << endl;
        }
        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg) {

            trigger.setDepthImage(pinhole_image_msg->image());
            trigger.setK(camera_matrix);
            trigger.setSensorOffset(offset);

            trigger.generateCloud();

            trigger.analyzeStructure();

            trigger.extractClusters();

            trigger.processClusters(argv[2]);
        }

    }

    return 0;
}
