#include <iostream>
#include <filesystem>
#include <ctime>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

#include <liblas/liblas.hpp>
#include <opencv4/opencv2/core.hpp>





#define SCALE 0.01
#define R 6378137.0
#define PI 3.14159265358979



typedef long long Timestamp;



using namespace std;
namespace fs = filesystem;





class Point {
public:
    double x, y, z;
    unsigned int intensity;

    Point(double x, double y, double z, unsigned int intensity) : x(x), y(y), z(z), intensity(intensity){}
    cv::Mat to4DMat() {
        cv::Mat mat = cv::Mat::ones(4, 1, CV_64F);
        mat.at<double>(0, 0) = this->x;
        mat.at<double>(1, 0) = this->y;
        mat.at<double>(2, 0) = this->z;

        return mat;
    }
    cv::Mat to3DMat() {
        cv::Mat mat = cv::Mat::zeros(3, 1, CV_64F);
        mat.at<double>(0, 0) = this->x;
        mat.at<double>(1, 0) = this->y;
        mat.at<double>(2, 0) = this->z;

        return mat;
    }
};

typedef struct Coord3D {
    double x, y, z;
} Coord3D;

typedef struct Image{
public:
    Timestamp timestamp;
} Image;

typedef struct VelodynePoints {
    vector<Point> points;

    Timestamp timestamp;
    Timestamp startTime;
    Timestamp endTime;
} VelodynePoints;

typedef struct Oxts {
public:
    double lat, lon, alt;
    double roll, pitch, yaw;
    double vn, ve, vf, vl, vu;
    double ax, ay, az;
    double af, al, au;
    double wx, wy, wz;
    double wf, wl, wu;
    double pos_accuracy, vel_accuracy;

    int navstat;
    int numsats;
    int posmode;
    int velmode;
    int orimode;

    Timestamp timestamp;
} Oxts;






int writeLas(vector<Point>, string);
vector<Point> readVelodynePoints(fs::path);
vector<Timestamp> readTimeStamps(fs::path);
string toString(int, int);
void printMatrix(cv::Mat);










class CalibImuToVelo {
public:
    CalibImuToVelo() {}

    void parse(fs::path filePath) {
        ifstream file(filePath);
        if (!file) {
            cout << filePath << " not found." << endl;
            exit(EXIT_FAILURE);
        }

        string date;
        getline(file, date);

        string line;
        string token;
        int index = 0;

        getline(file, line);
        istringstream issR(line);
        while (issR >> token) {
            try {
                double value = stod(token);
                this->r[index++] = value;
            }
            catch (invalid_argument const& e) {}
        }

        index = 0;
        getline(file, line);
        istringstream issT(line);
        while (issT >> token) {
            try {
                double value = stod(token);
                this->t[index++] = value;
            }
            catch (invalid_argument const& e) {}
        }

        file.close();

        this->setimuToVeloMat();
    }

    cv::Mat getimuToVeloMat() {
        return this->imuToVeloMat;
    }

private:
    double r[9];
    double t[3];
    cv::Mat imuToVeloMat;

    void setimuToVeloMat() {
        cv::Mat matrix = cv::Mat::zeros(4, 4, CV_64F);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                matrix.at<double>(i, j) = this->r[i*3 + j];

        for (int i = 0; i < 3; i++)
            matrix.at<double>(i, 3) = this->t[i];

        matrix.at<double>(3, 3) = 1.0;
        this->imuToVeloMat = matrix;
    }
};


class Records {
public:
    int numRecords;
    
    vector<Image> Images[4];
    vector<Oxts> oxtsData;
    vector<VelodynePoints> velodyneData;

    CalibImuToVelo calibImuToVelo;

    Records(string dirPath, string date, string drive) {
        SetDirectory(dirPath, date, drive);
    }

    void SetDirectory(string dirPath, string date, string drive) {
        fs::path directory(dirPath);
        directory /= date;
        fs::path rawDataDir(directory / (date+"_drive_"+drive+"_sync"));

        this->calibImuToVelo.parse(directory / "calib_imu_to_velo.txt");
        this->parseOxts(rawDataDir / "oxts");
        this->parseVelodynePoints(rawDataDir / "velodyne_points");

    }

    void parseOxts(fs::path directory) {
        // Get time stamps
        vector<Timestamp> timestamps = readTimeStamps(directory / "timestamps.txt");

        this->numRecords = timestamps.size();

        // Read OXTS data
        fs::path dataDirectory(directory / "data");
        for (int i = 0; i < timestamps.size(); i++) {

            fs::path path(dataDirectory / (toString(i, 10) + ".txt"));
            ifstream file(path);
            if (!file.is_open()) {
                cout << "cannot open " << path << "." << endl;
                exit(EXIT_FAILURE);
            }

            Oxts oxts;

            file >> oxts.lat >> oxts.lon >> oxts.alt;
            file >> oxts.roll >> oxts.pitch >> oxts.yaw;
            file >> oxts.vn >> oxts.ve >> oxts.vf >> oxts.vl >> oxts.vu;
            file >> oxts.ax >> oxts.ay >> oxts.az;
            file >> oxts.af >> oxts.al >> oxts.au;
            file >> oxts.wx >> oxts.wy >> oxts.wz;
            file >> oxts.wf >> oxts.wl >> oxts.wu;
            file >> oxts.pos_accuracy >> oxts.vel_accuracy;
            file >> oxts.navstat >> oxts.numsats >> oxts.posmode >> oxts.velmode >> oxts.orimode;

            oxts.timestamp = timestamps[i];

            this->oxtsData.push_back(oxts);
        }

    }

    void parseVelodynePoints(fs::path directory) {
        // Get time stamps
        vector<Timestamp> timestamps = readTimeStamps(directory / "timestamps.txt");
        vector<Timestamp> startTimes = readTimeStamps(directory / "timestamps_start.txt");
        vector<Timestamp> endTimes = readTimeStamps(directory / "timestamps_end.txt");

        // Read OXTS data
        fs::path dataDirectory(directory / "data");
        for (int i = 0; i < timestamps.size(); i++) {

            fs::path path(dataDirectory / (toString(i, 10) + ".bin"));
            vector<Point> points = readVelodynePoints(path);

            VelodynePoints velodynePoints;
            velodynePoints.points = points;

            velodynePoints.timestamp = timestamps[i];
            velodynePoints.startTime = startTimes[i];
            velodynePoints.endTime = endTimes[i];

            this->velodyneData.push_back(velodynePoints);
        }
    }

    int exportLas(string outPath) {

        vector<Point> newPoints;

        for (int i = 152; i < 153; i++) {
            // Oxts oxts = this->oxtsData[i];
            VelodynePoints velo = this->velodyneData[i];
            vector<Point> points = this->pointsToWorld(velo, i);

            newPoints.insert(newPoints.end(), points.begin(), points.end());
        }

        return writeLas(newPoints, outPath);
    }

private:

    cv::Mat ImuToWorldMat(Oxts oxts) {
        double s = cos(PI * this->oxtsData[0].lon / 180.0);
        double x = R * s * (PI * oxts.lon / 180.0);
        double y = R * s * log(tan(PI * (90 + oxts.lat) / 360.0));
        double z = oxts.alt;

        double sin_roll = sin(oxts.roll);
        double sin_pitch = sin(oxts.pitch);
        double sin_yaw = sin(oxts.yaw);

        double cos_roll = cos(oxts.roll);
        double cos_pitch = cos(oxts.pitch);
        double cos_yaw = cos(oxts.yaw);
        
        cv::Mat rotX = cv::Mat::zeros(3, 3, CV_64F);
        rotX.at<double>(0, 0) = 1.0;
        rotX.at<double>(1, 1) = cos_roll;
        rotX.at<double>(1, 2) = -sin_roll;
        rotX.at<double>(2, 1) = sin_roll;
        rotX.at<double>(2, 2) = cos_roll;

        cv::Mat rotY = cv::Mat::zeros(3, 3, CV_64F);
        rotY.at<double>(0 ,0) = cos_pitch;
        rotY.at<double>(0 ,2) = sin_pitch;
        rotY.at<double>(1 ,1) = 1.0;
        rotY.at<double>(2 ,0) = -sin_pitch;
        rotY.at<double>(2 ,2) = cos_pitch;
        
        cv::Mat rotZ = cv::Mat::zeros(3, 3, CV_64F);
        rotZ.at<double>(0 ,0) = cos_yaw;
        rotZ.at<double>(0 ,1) = -sin_yaw;
        rotZ.at<double>(1 ,0) = sin_yaw;
        rotZ.at<double>(1 ,1) = cos_yaw;
        rotZ.at<double>(2 ,2) = 1.0;

        cv::Mat rot = rotZ * rotY * rotX;
        
        cv::Mat matrix = cv::Mat::zeros(4, 4, CV_64F);

        for (int i = 0; i < 3; i++) 
            for (int j = 0; j < 3; j++)
                matrix.at<double>(i, j) = rot.at<double>(i, j);

        matrix.at<double>(0, 3) = x;
        matrix.at<double>(1, 3) = y;
        matrix.at<double>(2, 3) = z;
        matrix.at<double>(3, 3) = 1.0;

        // printMatrix(matrix);
        
        return matrix;
    }

    cv::Mat veloToImuMat() {
        return this->calibImuToVelo.getimuToVeloMat().inv();
    }

    cv::Mat pointsToMat(vector<Point> points) {
        int size = points.size();
        cv::Mat matrix(size, 3, CV_64F);

        for (int i = 0; i < size; i++) {
            Point point = points[i];

            matrix.at<double>(i, 0) = point.x;
            matrix.at<double>(i, 1) = point.y;
            matrix.at<double>(i, 2) = point.z;
        }

        return matrix;
    }

    vector<Point> pointsToWorld(VelodynePoints velo, int frame) {

        Oxts p_oxts, n_oxts, oxts = this->oxtsData[frame];
        double n_dlat, n_dlon, n_dalt, p_dlat, p_dlon, p_dalt;
        double n_droll, n_dpitch, n_dyaw, p_droll, p_dpitch, p_dyaw;

        if (frame > 0) {
            p_oxts = this->oxtsData[frame - 1];
            Timestamp dTime = oxts.timestamp - p_oxts.timestamp;

            p_dlat = (p_oxts.lat - oxts.lat) / dTime;
            p_dlon = (p_oxts.lon - oxts.lon) / dTime;
            p_dalt = (p_oxts.alt - oxts.alt) / dTime;
            p_droll = (p_oxts.roll - oxts.roll) / dTime;
            p_dpitch = (p_oxts.pitch - oxts.pitch) / dTime;
            p_dyaw = (p_oxts.yaw - oxts.yaw) / dTime;
        }
        if (frame < this->numRecords - 1) {
            n_oxts = this->oxtsData[frame + 1];
            Timestamp dTime = n_oxts.timestamp - oxts.timestamp;

            n_dlat = (n_oxts.lat - oxts.lat) / dTime;
            n_dlon = (n_oxts.lon - oxts.lon) / dTime;
            n_dalt = (n_oxts.alt - oxts.alt) / dTime;
            n_droll = (n_oxts.roll - oxts.roll) / dTime;
            n_dpitch = (n_oxts.pitch - oxts.pitch) / dTime;
            n_dyaw = (n_oxts.yaw - oxts.yaw) / dTime;
        }
        if (frame == 0) {
            p_dlat = -n_dlat;
            p_dlon = -n_dlon;
            p_dalt = -n_dalt;
            p_droll = -n_droll;
            p_dpitch = -n_dpitch;
            p_dyaw = -n_dyaw;
        }
        if (frame == this->numRecords - 1) {
            n_dlat = -p_dlat;
            n_dlon = -p_dlon;
            n_dalt = -p_dalt;
            n_droll = -p_droll;
            n_dpitch = -p_dpitch;
            n_dyaw = -p_dyaw;
        }

        
        cv::Mat veloToImu = this->veloToImuMat();
        vector<Point> points = velo.points;
        int numPoints = points.size();

        // position of velodyne
        Point posVelodyne(0, 0, 0, 255);
        points.push_back(posVelodyne);


        Timestamp startTime = velo.startTime;
        Timestamp dTime = (velo.endTime - startTime) / numPoints;
        vector<Point> newPoints;
        
        for (int i = 0; i < points.size(); i++) {
            Timestamp time = startTime + dTime * i;
            Point p = points[i];
            Oxts newOxts;

            Timestamp timeDiff = time - oxts.timestamp;
            if (timeDiff > 0) {
                newOxts.lat = oxts.lat + n_dlat * timeDiff;
                newOxts.lon = oxts.lon + n_dlon * timeDiff;
                newOxts.alt = oxts.alt + n_dalt * timeDiff;
                newOxts.roll = oxts.roll + n_droll * timeDiff;
                newOxts.pitch = oxts.pitch + n_dpitch * timeDiff;
                newOxts.yaw = oxts.yaw + n_dyaw * timeDiff;
            }
            else {
                timeDiff = -timeDiff;
                newOxts.lat = oxts.lat + p_dlat * dTime;
                newOxts.lon = oxts.lon + p_dlon * dTime;
                newOxts.alt = oxts.alt + p_dalt * dTime;
                newOxts.roll = oxts.roll + p_droll * dTime;
                newOxts.pitch = oxts.pitch + p_dpitch * dTime;
                newOxts.yaw = oxts.yaw + p_dyaw * dTime;
            }

            cv::Mat origin = this->ImuToWorldMat(oxts);
            cv::Mat newMat = this->ImuToWorldMat(newOxts);
            // printMatrix(newMat - origin);
            cv::Mat veloToWorld = newMat * veloToImu;
            cv::Mat pointMat = p.to4DMat();
            cv::Mat converted = veloToWorld * pointMat;

            Point newPoint(converted.at<double>(0,0), converted.at<double>(1,0), converted.at<double>(2,0), p.intensity);
            newPoints.push_back(newPoint);
        }

        return newPoints;
    }

    vector<Point> pointsToWorld(Oxts oxts, vector<Point> points) {
        cv::Mat veloToImu = this->veloToImuMat();
        cv::Mat imuToWorld = this->ImuToWorldMat(oxts);
        cv::Mat veloToWorld = imuToWorld * veloToImu;
        // cv::Mat veloToWorld = imuToWorld;
        
        int size = points.size();
        cv::Mat pointsMat = cv::Mat::ones(4, size, CV_64F);
        for (int i = 0; i < size; i++) {
            Point point = points[i];

            pointsMat.at<double>(0, i) = point.x;
            pointsMat.at<double>(1, i) = point.y;
            pointsMat.at<double>(2, i) = point.z;
        }

        cv::Mat worldsMat = veloToWorld * pointsMat;

        vector<Point> newPoints;
        for (int i = 0; i < size; i++) {
            double x = worldsMat.at<double>(0, i);
            double y = worldsMat.at<double>(1, i);
            double z = worldsMat.at<double>(2, i);

            Point newPoint{x, y, z, points[i].intensity};
            newPoints.push_back(newPoint);
        }

        return newPoints;

    }

    vector<Point> getWorldPoints(Oxts oxts, vector<Point> points) {
        // Coord3D coord = this->getWorldCoord(oxts);
        // cv::Mat transformed = this->getRotationMatrix(oxts) * this->pointsToMat(points).t();

        vector<Point> newPoints;
        // for (int i = 0; i < points.size(); i++) {
        //     double x = transformed.at<double>(0, i) + coord.x;
        //     double y = transformed.at<double>(1, i) + coord.y;
        //     double z = transformed.at<double>(2, i) + coord.z;

        //     Point newPoint{x, y, z, points[i].intensity};
        //     newPoints.push_back(newPoint);
        // }

        return newPoints;
    }

};


int writeLas(vector<Point> points, string outPath) {

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
        
    header.SetMax(maxX, maxY, maxZ);
    header.SetMin(minX, minY, minZ);
    header.SetRecordsCount(newPoints.size());

    ofstream file(outPath);
    liblas::Writer writer(file, header);

    for (liblas::Point point : newPoints) {
        writer.WritePoint(point);
    }

    return 1;
}


vector<Point> readVelodynePoints(fs::path path) {
    ifstream file(path);
    if (!file.is_open()) {
        cout << "cannot open " << path << "." << endl;
        exit(EXIT_FAILURE);
    }

    vector<Point> points;
    while (!file.eof()) {
        float x, y, z, intensity;

        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        file.read(reinterpret_cast<char*>(&intensity), sizeof(float));

        Point point((double)x, (double)y, (double)z, (unsigned int)(intensity * 255));
        points.push_back(point);
    }

    file.close();

    return points;
}

vector<Timestamp> readTimeStamps(fs::path path) {
    ifstream file(path);
    vector<Timestamp> timestamps;
    string line;

    while(getline(file, line)) {
        tm datetime = {};
        istringstream ss(line);

        ss >> get_time(&datetime, "%Y-%m-%d %H:%M:%S");

        if (ss.fail()) {
            cout << "wrong type of datetime" << endl;
            exit(EXIT_FAILURE);
        }

        time_t time = mktime(&datetime);
        if (time == -1) {
            cout << "wrong type of datetime" << endl;
            exit(EXIT_FAILURE);
        }

        Timestamp timestamp = stoull(line.substr(line.find('.')+1));
        timestamp += time * 1.0e+9;

        timestamps.push_back(timestamp);
        // 날자에 대한 기록은 x, 시간에 대한 기록만 남으니 주의해야함
    }

    return timestamps;
}

string toString(int number, int unit) {
    string numString = to_string(number);

    if (numString.length() < unit) {
        numString = string(unit - numString.length(), '0') + numString;
    }

    return numString;
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