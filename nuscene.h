#include <iostream>
#include <filesystem>
#include <ctime>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <unordered_map>

#include <liblas/liblas.hpp>
#include <opencv4/opencv2/core.hpp>

#include <nlohmann/json.hpp>

#include "TaskPool.hpp"


#define SCALE 0.001
#define HEIGHT_FILTER 0.05
#define LIDAR_HEIGHT 1.84
#define MAX_POINTS 100'000'000



using namespace std;
namespace fs = filesystem;
typedef long long Timestamp;






typedef struct Point {
    double x, y, z;
    int intensity, ringIndex;
} Point;

typedef struct Quaternion {
    double w, x, y, z;
} Quaternion;

typedef struct Coord3D {
    double x, y, z;
} Coord3D;

typedef struct EgoPose {
    string token;
    Timestamp timestamp;
    Quaternion rotation;
    Coord3D translation;
} EgoPose;

typedef struct CalibratedSensor {
    string token, sensor_token;
    Coord3D translation;
    Quaternion rotation;

} CalibratedSensor;

typedef struct Sample{
    string token;
    Timestamp timestamp;
    string prev, next, scene_token;
} Sample;

typedef struct SampleData {
    string token, sample_token, ego_pose_token, calibrated_sensor_token;
    Timestamp timestamp;
    string fileformat;
    bool is_key_frame;
    int height, width;
    string filename, prev, next;

} SampleData;


typedef struct egoPose {
    string token;
    Timestamp timestamp;
} egoPose;








nlohmann::json readJson(string);
int writePointclouds(vector<Point>&, string);
vector<Point> getPointsFromBin(string);
cv::Mat getRotationMat(const Quaternion&);
vector<Point> transformPoints(const Quaternion&, const Coord3D&, vector<Point>&, bool);

template<typename T>
void saveObjectToBin(T&, string);
template<typename T>
T loadObjectFromBin(string);




class Nuscene{
public:

    Nuscene() {}

    Nuscene(string path) {
        filesystem::path basePath(path);
        this->basePath = basePath;

        filesystem::path metaPath = basePath / "meta";

        // read ego_pose.json
        cout << "read ego_pose.json" << endl;
        filesystem::path egoPosePath = metaPath / "ego_pose.json";
        this->parseEgoPose(readJson(egoPosePath));

        // read sample.json
        cout << "read sample.json" << endl;
        filesystem::path samplePath = metaPath / "sample.json";
        this->parseSample(readJson(samplePath));

        // read sample_data.json
        cout << "read sample_data.json" << endl;
        filesystem::path sampleDataPath = metaPath / "sample_data.json";
        this->parseSampleData(readJson(sampleDataPath), true);

        // read calibrated_sensor.json
        cout << "read calibrated_sensor.json" << endl;
        filesystem::path calibratedSensorPath = metaPath / "calibrated_sensor.json";
        this->parseCalibratedSensor(readJson(calibratedSensorPath));
    }

    vector<SampleData> getInitSampleData() {
        vector<SampleData> sampleData;
        for (auto it = this->sampleData.begin(); it != this->sampleData.end(); it++) {
            SampleData target = it->second;
            if (target.prev.compare("") == 0) {
                sampleData.push_back(target);
            }
        }

        return sampleData;
    }

    void write(string token, string outPath) {
        filesystem::path outDir(outPath);
        this->writePointsFromSampleData(token, outDir / token, 0);
    }
    
    vector<Point> writePointsFromSampleData(string token, string outPath, int depth) {
        // cout << "search " << token << endl;
        vector<Point> points;

        auto sampleDataFinder = this->sampleData.find(token);
        if (sampleDataFinder == this->sampleData.end()) return points;

        SampleData sampleData = sampleDataFinder->second;

        // search next token
        string next = sampleData.next;
        if (next.compare("") != 0) {
            vector<Point> subPoints = this->writePointsFromSampleData(next, outPath, depth + 1);
            points.insert(points.end(), subPoints.begin(), subPoints.end());
        }

        // find ego_pose, calibrated_sensor rotation & translation
        string ego_pose_token = sampleData.ego_pose_token;
        string calibrated_sensor_token = sampleData.calibrated_sensor_token;
        auto egoPoseTokenFinder = this->egoPose.find(ego_pose_token);
        auto caliSensorFinder = this->calibratedSensor.find(calibrated_sensor_token);

        if (egoPoseTokenFinder != this->egoPose.end() && caliSensorFinder != this->calibratedSensor.end()) {
            Quaternion egoPoseRotation = egoPoseTokenFinder->second.rotation;
            Coord3D egoPoseTranslation = egoPoseTokenFinder->second.translation;
            Quaternion caliSensorRotation = caliSensorFinder->second.rotation;
            Coord3D caliSensorTranslation = caliSensorFinder->second.translation;

            // read points from lidar file
            string filename = sampleData.filename;
            vector<Point> myPoints = getPointsFromBin(this->basePath / filename);

            // transform points
            transformPoints(caliSensorRotation, caliSensorTranslation, myPoints, false);
            myPoints = transformPoints(egoPoseRotation, egoPoseTranslation, myPoints, true);
            points.insert(points.end(), myPoints.begin(), myPoints.end());
            
            // add position point of car
            points.push_back({egoPoseTranslation.x, egoPoseTranslation.y, egoPoseTranslation.z + LIDAR_HEIGHT, 255, 0});
        }

        if (points.size() > MAX_POINTS || depth == 0) {
            writePointclouds(points, outPath + "_" + to_string(depth) + ".las");
            points.clear();
        }

        return points;
    }


private:
    unordered_map<string, EgoPose> egoPose;
    unordered_map<string, SampleData> sampleData;
    unordered_map<string, CalibratedSensor> calibratedSensor;
    unordered_map<string, Sample> samples;

    filesystem::path basePath;

    void parseSampleData(nlohmann::json object, bool onlyLidar) {
        for (auto item : object) {
            if (onlyLidar) {
                string filename = item["filename"];
                if (filename.find(".bin") == string::npos) continue;
            }

            SampleData sample;
            sample.token = item["token"];
            sample.sample_token = item["sample_token"];
            sample.ego_pose_token = item["ego_pose_token"];
            sample.calibrated_sensor_token = item["calibrated_sensor_token"];
            sample.timestamp = item["timestamp"];
            sample.fileformat = item["fileformat"];
            sample.is_key_frame = item["is_key_frame"];
            sample.height = item["height"];
            sample.width = item["width"];
            sample.filename = item["filename"];
            sample.prev = item["prev"];
            sample.next = item["next"];
            
            this->sampleData[sample.token] = sample;
        }
    }

    void parseCalibratedSensor(nlohmann::json object) {
        for (auto item : object) {
            CalibratedSensor cSensor;
            cSensor.token = item["token"];
            cSensor.sensor_token = item["sensor_token"];

            nlohmann::json tObject = item["translation"];
            Coord3D translation{tObject[0], tObject[1], tObject[2]};
            cSensor.translation = translation;

            nlohmann::json rObject = item["rotation"];
            Quaternion rotation{rObject[0], rObject[1], rObject[2], rObject[3]};
            cSensor.rotation = rotation;

            this->calibratedSensor[cSensor.token] = cSensor;
        }
    }

    void parseEgoPose(nlohmann::json object) {
        for (auto item : object) {
            EgoPose egoPose;
            egoPose.token = item["token"];
            egoPose.timestamp = item["timestamp"];
            
            nlohmann::json tObject = item["translation"];
            Coord3D translation{tObject[0], tObject[1], tObject[2]};
            egoPose.translation = translation;

            nlohmann::json rObject = item["rotation"];
            Quaternion rotation{rObject[0], rObject[1], rObject[2], rObject[3]};
            egoPose.rotation = rotation;

            this->egoPose[egoPose.token] = egoPose;
        }
    }

    void parseSample(nlohmann::json object) {
        for (auto item : object) {
            Sample sample;
            sample.token = item["token"];
            sample.timestamp = item["timestamp"];
            sample.prev = item["prev"];
            sample.next = item["next"];
            sample.scene_token = item["scene_token"];

            this->samples[sample.token] = sample;
        }
    }
};



nlohmann::json readJson(string path) {
    ifstream file(path);
    if (!file.is_open()) {
        cout << "cannot open " << path << "." << endl;
        exit(EXIT_FAILURE);
    }

    string contents((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
    file.close();

    nlohmann::json object = nlohmann::json::parse(contents);
    return object;
}


int writePointclouds(vector<Point> &points, string outPath) {
    if (points.size() == 0) return 0;

    double minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    double maxX = FLT_MIN, maxY = FLT_MIN, maxZ = FLT_MIN;
    vector<liblas::Point> newPoints;
    liblas::Header header;

    header.SetScale(SCALE, SCALE, SCALE);

    for (int j = 0; j < points.size(); j++) {
        Point point = points[j];
        double x = point.x;
        double y = point.y;
        double z = point.z;

        liblas::Point newPoint(&header);
        newPoint.SetCoordinates(x, y, z);
        newPoint.SetIntensity(point.intensity);
        // newPoint.SetTime()
        newPoints.push_back(newPoint);

        x = newPoint.GetX();
        y = newPoint.GetY();
        z = newPoint.GetZ();
        
        if (x < minX) minX = x;
        if (y < minY) minY = y;
        if (z < minZ) minZ = z;

        if (x > maxX) maxX = x;
        if (y > maxY) maxY = y;
        if (z > maxZ) maxZ = z;
    }
        
    header.SetMax(maxX + SCALE*10, maxY + SCALE*10, maxZ + SCALE*10);
    header.SetMin(minX - SCALE*10, minY - SCALE*10, minZ - SCALE*10);
    header.SetRecordsCount(newPoints.size());

    ofstream file(outPath);
    liblas::Writer writer(file, header);

    for (liblas::Point point : newPoints) {
        writer.WritePoint(point);
    }

    return 1;
}

vector<Point> getPointsFromBin(string path) {
    vector<Point> points;

    ifstream file(path, ios::binary);
    if (!file.is_open()) {
        cout << "cannot open " << path << endl;
        return points;
    }
    
    while (!file.eof()) {
        float x, y, z, intensity, ringIndex;

        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
        file.read(reinterpret_cast<char*>(&ringIndex), sizeof(int));

        Point point{(double)x, (double)y, (double)z, (int)intensity, (int)ringIndex};
        points.push_back(point);
    }

    file.close();
    return points;
}

void printMatrix(cv::Mat mat) {
    int rows = mat.rows;
    int cols = mat.cols;

    cout << "" << endl;
    for (int i = 0; i < rows; i++) {
        cout << "[";
        for (int j = 0; j < cols; j++) {
            cout << " " << mat.at<double>(i, j);
        }
        cout << "]" << endl;
    }
    cout << "" << endl;
}


cv::Mat getRotationMat(const Quaternion& q) {
    cv::Mat mat(3, 3, CV_64F);
    
    mat.at<double>(0, 0) = 2 * (q.w * q.w + q.x * q.x) - 1;
    mat.at<double>(0, 1) = 2 * (q.x * q.y - q.w * q.z);
    mat.at<double>(0, 2) = 2 * (q.x * q.z + q.w * q.y);

    mat.at<double>(1, 0) = 2 * (q.x * q.y + q.w * q.z);
    mat.at<double>(1, 1) = 2 * (q.w * q.w + q.y * q.y) - 1;
    mat.at<double>(1, 2) = 2 * (q.y * q.z - q.w * q.x);

    mat.at<double>(2, 0) = 2 * (q.x * q.z - q.w * q.y);
    mat.at<double>(2, 1) = 2 * (q.y * q.z + q.w * q.x);
    mat.at<double>(2, 2) = 2 * (q.w * q.w + q.z * q.z) - 1;

    return mat;
}

vector<Point> transformPoints(const Quaternion& rotation, const Coord3D& translation, vector<Point>& points, bool filter) {
    vector<Point> newPoints;
    int size = points.size();
    if (size == 0) return newPoints;

    // create points matrix
    cv::Mat pointsMat(3, size, CV_64F);
    for (int i = 0; i < size; i++) {
        pointsMat.at<double>(0, i) = points[i].x;
        pointsMat.at<double>(1, i) = points[i].y;
        pointsMat.at<double>(2, i) = points[i].z;
    }

    // rotate points
    pointsMat = getRotationMat(rotation) * pointsMat;

    // translation points
    for (int i = 0; i < size; i++) {
        if (filter && pointsMat.at<double>(2, i) > HEIGHT_FILTER) continue;

        points[i].x = pointsMat.at<double>(0, i) + translation.x;
        points[i].y = pointsMat.at<double>(1, i) + translation.y;
        points[i].z = pointsMat.at<double>(2, i) + translation.z;
        newPoints.push_back(points[i]);
    }

    return newPoints;
}

template<typename T>
void saveObjectToBin(T& object, string outPath) {
    ofstream file(outPath, ios::out | ios::app | ios::binary);
    if (!file.is_open()) {
        cout << "cannot write " << outPath << endl;
        exit(EXIT_FAILURE);
    }

    file.write(reinterpret_cast<const char*>(&object), sizeof(object));
    file.close();
}

template<typename T>
T loadObjectFromBin(string path) {
    T object;

    ifstream file(path, ios::binary);
    if (!file.is_open()) {
        cout << "cannot open " << path << endl;
        exit(EXIT_FAILURE);
    }

    file.read(reinterpret_cast<char*>(&object), sizeof(object));
    file.close();

    return object;
}