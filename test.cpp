#include <iostream>
// #include "tracklets.h"
// #include "kitti.h"
#include "nuscene.h"
#include "TaskPool.hpp"


#define TRHEAD_NUM 4


using namespace std;
void testNuscene(string path,int num);
int testBlob();
int testMini();

int main() {
    testMini();
}

int testBlob() {
    testNuscene("/nuscene/v1.0-trainval01_blobs", 8);
}

int testMini() {
    testNuscene("/nuscene/v1.0-mini", 0);
}


void testNuscene(string path,int num) {
    Nuscene nuscene(path);
    auto samples = nuscene.getInitSampleData();

    filesystem::path baseDir(path);

    struct Task {
        string token;
        string outPath;
        Nuscene *nuscene;
    };
    auto processor = [](shared_ptr<Task> task) {
        task->nuscene->write(task->token, task->outPath);
    };
    TaskPool<Task> pool(TRHEAD_NUM, processor);

    int size = samples.size();
    if (num > 0 && num << size) {
        size = num;
    }

    for (int i = 0; i < size; i ++) {
        cout << samples[i].filename << endl;

        shared_ptr<Task> task = make_shared<Task>();
        task->nuscene = &nuscene;
        task->token = samples[i].token;
        task->outPath = baseDir / "test";
        pool.addTask(task);
    }

    pool.waitTillEmpty();
    pool.close();
}




// int test2() {
//     string filePath = "/Documents/github/data/kitti_dataset/raw";
//     Records records(filePath, "2011_09_26", "0101");
//     records.exportLas("/Documents/github/data/kitti_dataset/raw/test.las");
// }

// int test3() {
//     string filePath = "/Downloads/data_road_velodyne/training/velodyne/um_000002.bin";
//     string outPath = "/Downloads/data_road_velodyne/training/velodyne/alplha.las";

//     auto points = readVelodynePoints(filePath);
//     writeLas(points, outPath);
// }

// int test() {
//     Tracklets *tracklets = new Tracklets();
//     string filePath = "/Downloads/새 폴더/2011_09_26_drive_0015_sync/tracklet_labels.xml";
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