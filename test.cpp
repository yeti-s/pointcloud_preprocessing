#include <iostream>
// #include "tracklets.h"
#include "kitti.h"

using namespace std;
int test();
int test2();


int main() {
    test2();
}

int test2() {
    string filePath = "/mnt/c/Users/yeti/Documents/github/data/kitti_dataset/raw";
    Records records(filePath, "2011_09_26", "0015");
    records.exportLas("/mnt/c/Users/yeti/Documents/github/data/kitti_dataset/raw/test.las");
}

// int test() {
//     Tracklets *tracklets = new Tracklets();
//     string filePath = "/mnt/c/Users/yeti/Downloads/새 폴더/2011_09_26_drive_0015_sync/tracklet_labels.xml";
//     try {
//         tracklets->loadFromFile(filePath);
//         int numTracklets = tracklets->numberOfTracklets();
//         cout << "num tracklets : " << numTracklets << endl;

//         int numRecoreded = 0;

//         for (int i = 0; i < numTracklets; i++) {
//             auto tracklet = tracklets->getTracklet(i);
//             cout << i << "th tracklets, " << tracklet->first_frame << ", " << tracklet->poses.size() << " poses" << endl;
//             numRecoreded += tracklet->poses.size();
//         }

//         cout << "total record : " << numRecoreded << endl;

//         // auto tmp = tracklets->getTracklet(0);
//         // auto poses = tmp->poses;
//         // for (auto it = poses.begin(); it != poses.end(); it++) {
//         //     cout << endl;
//         //     cout << it->rx << " " << it->ry << " " << it->rz << endl;
//         //     cout << it->tx << " " << it->ty << " " << it->tz << endl;
//         // }
//         // cout << tracklets->
//     } catch (exception &e) {

//     }
//     delete tracklets;
// }