#include "slam.h"

#include <string.h>
#include <string>
#include <iostream>
#include "json/json.h"
#include "src/stdafx.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <string>
#include "stdio.h"
#include <iostream>
using namespace std;
#include "include/httpClient.h"
#include "curl/curl.h"
#include "curl/easy.h"
#include <ctime>
#define MAX_FACE 100
#define MAX_PERSON 1000
std::string strResult;

std::string HTTPRESULT = "";

char followID[80];
int followflag = 0;
ImageDB imgdb;

class ProgramRunTimer
{
    static const int kClockPerSecond = CLOCKS_PER_SEC; // 每秒时钟的跳数
public:
    ProgramRunTimer(void):cost_time_(0),start_time_(0),end_time_(0)
    {
        Start();
    };
    ~ProgramRunTimer(void){};

    void Reset(void)
    {
        cost_time_ = 0;
        start_time_ = 0;
        end_time_ = 0;
    }

    void Start(void)
    {
        start_time_ = clock();
        return;
    }

    void End(void)
    {
        end_time_ = clock();
        cost_time_ = (end_time_ - start_time_) / (kClockPerSecond);
        return;
    }

    double Cost(void)
    {
        return cost_time_;
    }
protected:
    double cost_time_;
    clock_t start_time_;
    clock_t end_time_;
};

struct Face{
    char faceID[80];
    int ageValue;
    int ageRange;
    int gender;
    int race;
    float centerX;
    float centerY;
    float height;
    float width;
    void print();
    void init();
};

struct Person{
    char personID[80];
    void init();
};

struct FaceSet{
    Face set[MAX_FACE];
    int faceNumber;
    void print();
    void deserializeToObj(const string& strJson);
};

struct PersonSet{
   Person set[MAX_PERSON];
   int personNumber;
   void deserializeToObj(const string& strJson);
};

//string serializeToJson(const Student& student);

//@brief:将给定的学生对象序列化为json字符串
//@param:student:学生对象
//@ret:json字符串
/*string serializeToJson(const Student& student){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value person;

    person["ID"] = student.ID;
    person["name"] = student.name;
    person["age"]=student.age;
    person["gender"]=student.gender;
    person["major"]=student.major;
    root.append(person);

    string strJson=writer.write(root);
    return strJson;
}*/

void Face::print(){
    cout << " centerX: " << centerX << " centerY: " << centerY << " width: "<< width << " height: "<< height << endl;
}

void Face::init(){
    memset(faceID, 0, sizeof(char)*80);
}

void FaceSet::print(){
    for (int i = 0; i < faceNumber; i++){
        cout << "faceset id: " << i;
        set[i].print();
    }
}

static int writer(char *data, size_t size, size_t nmemb, std::string *writerData) {
    if(writerData == NULL) return 0;

    writerData->append(data, size * nmemb);

    return size * nmemb;
}

void Person::init(){
    memset(personID,0, sizeof(char)*80);
}

void FaceSet::deserializeToObj(const string& strJson){
    Json::Reader reader;
    Json::Value value;
    FaceSet faceSet;
    for (int i = 0; i < MAX_FACE; i++)
        set[i].init();

    if (reader.parse(strJson, value)){
        faceNumber = value["face"].size();
        for(int i = 0; i < value["face"].size(); i++){
             set[i].centerX = value["face"][i]["position"]["center"]["x"].asFloat();
             set[i].centerY = value["face"][i]["position"]["center"]["y"].asFloat();
             set[i].height= value["face"][i]["position"]["height"].asFloat();
             set[i].width= value["face"][i]["position"]["width"].asFloat();
             strcpy(set[i].faceID,value["face"][i]["face_id"].asString().c_str());
        }
       // strcpy(student.name,value["name"].asString().c_str());
      //  student.age=value["age"].asInt();
    }
}

void PersonSet::deserializeToObj(const string& strJson){
    Json::Reader reader;
    Json::Value value;
    for (int i = 0; i < MAX_PERSON; i++){
        set[i].init();
    }
    if (reader.parse(strJson, value)){
        personNumber = value["person"].size();
        for(int i = 0; i < value["person"].size(); i++){
             strcpy(set[i].personID,value["person"][i]["person_id"].asString().c_str());
        }
    }
}

void createPerson(Face face){
    HTTPRESULT = "";
    CURL *curl = curl_easy_init();
    CURLcode res = curl_global_init(CURL_GLOBAL_WIN32);

    struct curl_httppost *formpost=NULL;
    struct curl_httppost *lastptr=NULL;

    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_key",
        CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_secret",
        CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
        CURLFORM_END);
    curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/person/create");
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

    char error[1024];
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

    res = curl_easy_perform(curl);
    if(res != CURLE_OK) cout<<endl<<error<<endl;

    curl_easy_cleanup(curl);
    curl_formfree(formpost);
    cout<<HTTPRESULT<<endl;

    char newpersonID[80];
    memset(newpersonID, 0, sizeof(char)*80);
    Json::Reader reader;
    Json::Value value;
    if (reader.parse(HTTPRESULT, value)){
          strcpy(newpersonID,value["person_id"].asString().c_str());
    }

    cout << "debug newpersonID" << newpersonID << endl;
    //
    if (followflag == 0){
        for(int i = 0; i < 80; i++){
            followID[i] = newpersonID[i];
        }
        followflag = 1;
    }
     //添加人脸
    curl = curl_easy_init();
    res = curl_global_init(CURL_GLOBAL_WIN32);

    formpost=NULL;
    lastptr=NULL;
    HTTPRESULT = "";
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_key",
        CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_secret",
        CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "person_id",
        CURLFORM_COPYCONTENTS, newpersonID,
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "face_id",
        CURLFORM_COPYCONTENTS, face.faceID,
        CURLFORM_END);
    cout << "debug face_id: " << face.faceID << endl;
    curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/person/add_face");
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

    res = curl_easy_perform(curl);
    if(res != CURLE_OK) cout<<endl<<error<<endl;

    curl_easy_cleanup(curl);
    curl_formfree(formpost);
    cout<<"add face result: " << HTTPRESULT<<endl;

    //训练人脸
    curl = curl_easy_init();
    res = curl_global_init(CURL_GLOBAL_WIN32);

    formpost=NULL;
    lastptr=NULL;
    HTTPRESULT = "";
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_key",
        CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_secret",
        CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "person_id",
        CURLFORM_COPYCONTENTS, newpersonID,
        CURLFORM_END);
    curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/train/verify");
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

    res = curl_easy_perform(curl);
    if(res != CURLE_OK) cout<<endl<<error<<endl;

    curl_easy_cleanup(curl);
    curl_formfree(formpost);
    cout<<"return train/verify: " << HTTPRESULT<<endl;

    cout << "debug " << endl;
    char newSessionID[80];
    memset(newSessionID, 0, sizeof(char)*80);
    if (reader.parse(HTTPRESULT, value)){
          strcpy(newSessionID,value["session_id"].asString().c_str());
    }
    cout << "debug SessionId" << newSessionID << endl;

    ProgramRunTimer* myTimer = new ProgramRunTimer();
    //getSession
    while (true){
        formpost=NULL;
        lastptr=NULL;
        HTTPRESULT = "";
        curl = curl_easy_init();
        res = curl_global_init(CURL_GLOBAL_WIN32);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "api_key",
            CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "api_secret",
            CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "session_id",
            CURLFORM_COPYCONTENTS, newSessionID,
            CURLFORM_END);
        cout << "debug face_id: " << face.faceID << endl;
        curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/info/get_session");
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK) cout<<endl<<error<<endl;

        curl_easy_cleanup(curl);
        curl_formfree(formpost);
        cout<<HTTPRESULT<<endl;

        char Status[80];
        memset(Status, 0, sizeof(char)*80);
        if (reader.parse(HTTPRESULT, value)){
              strcpy(Status,value["status"].asString().c_str());
        }
        cout << "debug Status" << Status << endl;
        if (Status[0] == 'S'){
            break;
        }
    }
    myTimer->End();
    cout << "use time" << myTimer->Cost() << endl;
    delete myTimer;
}

bool hasface(Person person, Face face){
    HTTPRESULT = "";
    CURL *curl = curl_easy_init();
    CURLcode res = curl_global_init(CURL_GLOBAL_WIN32);

    struct curl_httppost *formpost=NULL;
    struct curl_httppost *lastptr=NULL;

    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_key",
        CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_secret",
        CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "person_id",
        CURLFORM_COPYCONTENTS, person.personID,
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "face_id",
        CURLFORM_COPYCONTENTS, face.faceID,
        CURLFORM_END);
    curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/recognition/verify");
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

    char error[1024];
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

    res = curl_easy_perform(curl);
    if(res != CURLE_OK) cout<<endl<<error<<endl;

    curl_easy_cleanup(curl);
    curl_formfree(formpost);
    cout<<HTTPRESULT<<endl;

    char judge[80];
    memset(judge, 0, sizeof(char)*80);
    Json::Reader reader;
    Json::Value value;
//    float confidence = 0;
    if (reader.parse(HTTPRESULT, value)){
        strcpy(judge,value["is_same_person"].asString().c_str());
//        confidence = value["confidence"].asFloat();
    }
    //这里confidence是置信度，可以调整是否接受
    if (judge[0] == 't'){   //如果已知
        curl = curl_easy_init();
        res = curl_global_init(CURL_GLOBAL_WIN32);

        formpost=NULL;
        lastptr=NULL;
        HTTPRESULT = "";
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "api_key",
            CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "api_secret",
            CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "person_id",
            CURLFORM_COPYCONTENTS, person.personID,
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "face_id",
            CURLFORM_COPYCONTENTS, face.faceID,
            CURLFORM_END);
        cout << "debug face_id: " << face.faceID << endl;
        curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/person/add_face");
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK) cout<<endl<<error<<endl;

        curl_easy_cleanup(curl);
        curl_formfree(formpost);
        cout<<"add face result: " << HTTPRESULT<<endl;

        //训练人脸
        curl = curl_easy_init();
        res = curl_global_init(CURL_GLOBAL_WIN32);

        formpost=NULL;
        lastptr=NULL;
        HTTPRESULT = "";
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "api_key",
            CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "api_secret",
            CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
            CURLFORM_END);
        curl_formadd(&formpost,
            &lastptr,
            CURLFORM_COPYNAME, "person_id",
            CURLFORM_COPYCONTENTS, person.personID,
            CURLFORM_END);
        curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/train/verify");
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

        res = curl_easy_perform(curl);
        if(res != CURLE_OK) cout<<endl<<error<<endl;

        curl_easy_cleanup(curl);
        curl_formfree(formpost);
        cout<<"return train/verify: " << HTTPRESULT<<endl;

        cout << "debug " << endl;
        char newSessionID[80];
        memset(newSessionID, 0, sizeof(char)*80);
        if (reader.parse(HTTPRESULT, value)){
              strcpy(newSessionID,value["session_id"].asString().c_str());
        }
        cout << "debug SessionId" << newSessionID << endl;

        ProgramRunTimer* myTimer = new ProgramRunTimer();
        //getSession
        while (true){
            formpost=NULL;
            lastptr=NULL;
            HTTPRESULT = "";
            curl = curl_easy_init();
            res = curl_global_init(CURL_GLOBAL_WIN32);
            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "api_key",
                CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
                CURLFORM_END);
            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "api_secret",
                CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
                CURLFORM_END);
            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "session_id",
                CURLFORM_COPYCONTENTS, newSessionID,
                CURLFORM_END);
            cout << "debug face_id: " << face.faceID << endl;
            curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/info/get_session");
            curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
            curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
            curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

            curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

            res = curl_easy_perform(curl);
            if(res != CURLE_OK) cout<<endl<<error<<endl;

            curl_easy_cleanup(curl);
            curl_formfree(formpost);
            cout<<HTTPRESULT<<endl;

            char Status[80];
            memset(Status, 0, sizeof(char)*80);
            if (reader.parse(HTTPRESULT, value)){
                  strcpy(Status,value["status"].asString().c_str());
            }
            cout << "debug Status" << Status << endl;
            if (Status[0] == 'S'){
                break;
            }
        }
        myTimer->End();
        cout << "use time" << myTimer->Cost() << endl;
        delete myTimer;
        return 1;
    }
    return 0;
}

PersonSet GetAllPerson(){
    PersonSet personSet;
    CURL *curl = curl_easy_init();
    CURLcode res = curl_global_init(CURL_GLOBAL_WIN32);
    HTTPRESULT = "";

    struct curl_httppost *formpost=NULL;
    struct curl_httppost *lastptr=NULL;

    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_key",
        CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
        CURLFORM_END);
    curl_formadd(&formpost,
        &lastptr,
        CURLFORM_COPYNAME, "api_secret",
        CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
        CURLFORM_END);
    curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/info/get_person_list");
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

    char error[1024];
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

    res = curl_easy_perform(curl);
    if(res != CURLE_OK) cout<<endl<<error<<endl;

    curl_easy_cleanup(curl);
    curl_formfree(formpost);
    cout<<HTTPRESULT<<endl;

    personSet.deserializeToObj(HTTPRESULT);
    return personSet;
}


void* slam_event(void* args) {
    /******************************************************************************/
    //launch kinect
    
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    
    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return NULL;
    }
    
    std::string serial = "";
    
    bool enable_rgb = true;
    bool enable_depth = true;
    
    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    
    pipeline = new libfreenect2::CpuPacketPipeline();
    
    dev = freenect2.openDevice(serial , pipeline);
    
    int types = 0;
    if (enable_rgb)
    {
        types |= libfreenect2::Frame::Color;
    }
    if (enable_depth)
    {
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    }
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    
    if (enable_rgb && enable_depth)
    {
        if (!dev->start())
        {
            return NULL;
        }
    }
    else
    {
        if (!dev->startStreams(enable_rgb , enable_depth))
        {
            return NULL;
        }
    }

    // 相机内参
#if USE_IR
    libfreenect2::Freenect2Device::IrCameraParams camera_params = dev->getIrCameraParams();
#else
    libfreenect2::Freenect2Device::ColorCameraParams camera_params = dev->getColorCameraParams();
    assert_e(false);
#endif

    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams rgb_params = dev->getColorCameraParams();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    
    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams() , dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512 , 424 , 4) , registered(512 , 424 , 4);


#if USEg2o
    /******************************* 
    // 新增:有关g2o的初始化
    *******************************/
    // 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver ); 
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

#endif

    int count = 0;
    srand(unsigned(int(NULL)));

    ImageList *current;//, *last;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr base(new pcl::PointCloud<pcl::PointXYZRGB>);
    uint32_t background = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
    
    struct timeval tt1, tt2, tt3, tt4, tt10, tt11;

    imgdb.C.cx = camera_params.cx;
    imgdb.C.cy = camera_params.cy;
    imgdb.C.fx = camera_params.fx;
    imgdb.C.fy = camera_params.fy;
    imgdb.C.scale = 1000.0;

        // 滤波器
    pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("x");
    pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

    double gridsize = 0.01;
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    FILE *fp;

    // 局部镜头(显示障碍物信息)
    // 全局镜头(显示点云拼接信息)
#if USE_VIEWER
    boost::shared_ptr<pcl::visualization::PCLVisualizer> local_viewer(new pcl::visualization::PCLVisualizer("Local Viewer")),
                                                         global_viewer(new pcl::visualization::PCLVisualizer("Global Viewer"));
    imgdb.setViewer(local_viewer, global_viewer);
    pcl::visualization::Camera camera;
    local_viewer->getCameraParameters(camera);
    //printf("focal(%lf,%lf,%lf) view(%lf,%lf,%lf) pos(%lf,%lf,%lf)\n",
    //       camera.focal[0], camera.focal[1], camera.focal[2],
    //       camera.view[0], camera.view[1], camera.view[2],
    //       camera.pos[0], camera.pos[1], camera.pos[2]);
    fp = fopen("../camera.txt", "r");
    if (fp) {
        fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &camera.focal[0], &camera.focal[1], &camera.focal[2],
                                     &camera.pos[0], &camera.pos[1], &camera.pos[2],
                                     &camera.view[0], &camera.view[1], &camera.view[2]);
        fclose(fp);
    }
    local_viewer->setCameraParameters(camera);
    global_viewer->setCameraParameters(camera);
#endif

    int max_iters;
    fp = fopen("../params.txt", "r");
    if (fp) {
        fscanf(fp, "%f %f %f %f %d %d", &voxel_resolution, &seed_resolution, &posdist, &negdist, &min_clusters_contains, &max_iters);
        assert_e(posdist > negdist);
        fclose(fp);
    }


    while (count++ < max_iters) {
        if (!listener.waitForNewFrame(frames , 30 * 1000)) {
            std::cout << "timeout!" << std::endl;
            break;
        }
        else {

            gettimeofday(&tt1, NULL);
            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
             current = imgdb.add(); current->time = tt1;
             current->collect_x = imgdb.robot->getX();
             current->collect_y = imgdb.robot->getY();
            registration->apply(rgb , depth , &undistorted , &registered);

            /*gettimeofday(&tt10, NULL);

            cv::Mat rgb_img(1080, 1920, CV_8UC4, rgb->data);
            imwrite("/home/chengkai/test/person2_2.jpg", rgb_img);

            cout << "followflag -------------------------------------------------" << followflag << endl;
            CURL *curl = curl_easy_init();
            CURLcode res = curl_global_init(CURL_GLOBAL_WIN32);
            HTTPRESULT = "";
            struct curl_httppost *formpost=NULL;
            struct curl_httppost *lastptr=NULL;
            //        struct curl_slist *headerlist=NULL;
            //        static const char buf[] = "Expect:";

            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "api_key",
                CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
                CURLFORM_END);
            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "api_secret",
                CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
                CURLFORM_END);

            char* file_data = NULL;
            long file_size = 0;
            FILE* fp = fopen("/home/chengkai/test/person2_2.jpg","rb");
            cout << "reading file" << endl;
            if (fp)
            {
                cout<< "reading successfully" << endl;
                fseek(fp, 0, SEEK_END);
                file_size = ftell(fp);
                cout << file_size << endl;
                fseek(fp, 0, SEEK_SET);
                file_data = new char[file_size+1];
                fread(file_data,1,file_size,fp);
                fclose(fp);
            }


            curl_formadd(&formpost, &lastptr,
                CURLFORM_COPYNAME, "img",
                CURLFORM_BUFFER, "p2.jpg",
                CURLFORM_BUFFERPTR, file_data,
                CURLFORM_BUFFERLENGTH, file_size,
                CURLFORM_CONTENTTYPE, "image/jpeg",
                CURLFORM_END);

            if(curl) {
                cout << "start" << endl;
                curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/detection/detect");
                curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
                curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
                curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

                char error[1024];
                curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

                res = curl_easy_perform(curl);
                if(res != CURLE_OK) cout<<endl<<error<<endl;
            }

            curl_easy_cleanup(curl);
            curl_formfree(formpost);
         //   cout<<HTTPRESULT<<endl;

            FaceSet faceSet;
            faceSet.deserializeToObj(HTTPRESULT);
            faceSet.print();
            if(file_data != NULL)
                delete [] file_data;

             for (int i = 0; i < faceSet.faceNumber; i++){
                 PersonSet personSet = GetAllPerson();
                 int flag = 0;
                 int isfollow = 0;
                 for (int j = 0; j < personSet.personNumber; j++){
                     if (hasface(personSet.set[j],faceSet.set[i])){
                        flag = 1;
                        if (followflag == 1){
                               cout << "followID----------------------------------" << followID << endl;
                               cout << "personSet.set--------------------------" <<  personSet.set[j].personID << endl;
                               for (int k = 0; k < 10; k++){
                                    if (followID[k] != personSet.set[j].personID[k]){
                                        isfollow = 1;
                                        break;
                                    }
                               }
                               cout << "isfollow---------------------------" << isfollow << endl;
                               if (isfollow == 0){
                                    float x = faceSet.set[i].centerX;
                                    float y = faceSet.set[i].centerY;
                                    float h = faceSet.set[i].height;
                                    float w = faceSet.set[i].width;
                                    int *mydata = (int*)rgb->data;
                                    for (int p = 0; p < 1080; p++) {
                                        for (int q = 0; q < 1920; q++) {
//                                            if (p < 10.8 * (x + w/2) && p > 10.8 * (x - w /2) && q < 19.2 * (y + h/2) && q > 19.2 * (y - h/2)){
                                             if (p < 10.8 * (y + h/2) && p > 10.8 * (y - h /2) && q < 19.2 * (x + w/2) && q > 19.2 * (x - w/2)){
                                                mydata[p* 1920 + q] = mydata[p* 1920 + q] & (0xF0FF);
                                            }
                                        }
                                    }
                                    //cv::Mat test_img(1080, 1920, CV_8UC4,rgb->data);
                                    //imwrite("/home/chengkai/test/test.jpg", test_img);
 uint32_t *rgb_raw = (uint32_t*)rgb->data;
                                    float *depth_raw = (float*)depth->data;
                                    registration_qby->apply(rgb , depth , &undistorted_qby , &registered_qby);
                                    pcl::PointXYZRGB point;
                                    float point_depth;
                                    for (int i = 0; i < 424; i++) {
                                        int loopflag = 0;
                                        for (int j = 0; j < 512; j++) {
                                            if (i < 4.24 * (y + h/2) && i > 4.24 * (y - h /2) && j < 5.12 * (x + w/2) && j > 5.12 * (x - w/2)){
                                                registration_qby->getPointXYZRGB(&undistorted_qby, &registered_qby, i, j,
                                                    point.x, point.y, point.z, point.rgb);
                                                uint32_t color =  (*reinterpret_cast<int*>(&point.rgb) >> 16) & 0xFF;
                                                if (color == 0){
                                                    point_depth = depth_raw[i*512+j];
                                                    loopflag = 1;
                                                    break;
                                                }
                                                //if (isnan(point.rgb)) point.rgb = *reinterpret_cast<float*>(&background);
                                            }
                                        }
                                        if (loopflag == 1){
                                             break;
                                        }
                                    }



                                    /*registration_qby->apply(rgb , depth , &undistorted_qby , &registered_qby);
                                    pcl::PointXYZRGB point;
                                    for (int i = 0; i < 424; i++) {
                                        for (int j = 0; j < 512; j++) {
                                            registration_qby->getPointXYZRGB(&undistorted_qby, &registered_qby, i, j,
                                                point.x, point.y, point.z, point.rgb);
                                            //if (isnan(point.rgb)) point.rgb = *reinterpret_cast<float*>(&background);
                                        }
                                    }
                                    /*
                                    int get_depth = 0;
                                    int total = 0;
                                    for(int p = (int)((10.8*(y - h/2)) - 10); p < (int)(10.8 * (y + h/2)) + 10; p++){
                                        for(int q =  (int)((19.2*(x - w/2)) - 10); q < (int)(19.2 * (x + w/2)) + 10; q++){
                                            if ( == ){
                                                get_depth += ;
                                                total++;
                                            }
                                        }
                                    }*/
                               /*}
                        }
                        break;
                     }
                 }
                 if (flag == 0){             //如果不在已知集合中，需要创建人物
                     createPerson(faceSet.set[i]);
                 }
             }
gettimeofday(&tt11, NULL);
                printf("Time used %d for generate point cloud = %lf\n", 233, gettime(tt10, tt11));*/



            assert_e(count == current->id + 1);

            if (1) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = current->cloud;
                cv::Mat *img = current->img,
                        *depth_img = current->depth;
                
                float *ir_raw = (float*)ir->data,
                      *depth_raw = (float*)depth->data;
                
#if USE_IR
                float max_ir = 0;
                for (int i = 0; i < 424 * 512; i++) {
                    if (depth_raw[i] > max_ir) max_ir = depth_raw[i];
                    //if (ir_raw[i] > max_ir) max_ir = ir_raw[i];
                }
#endif

                pcl::PointXYZRGB point;
                for (int i = 0; i < 424; i++) {
                    for (int j = 0; j < 512; j++) {
                        registration->getPointXYZRGB(&undistorted, &registered, i, j,
                            point.x, point.y, point.z, point.rgb);
                        if (isnan(point.rgb)) point.rgb = *reinterpret_cast<float*>(&background);
#if USE_IR
                        img->at<char>(i,j) = int(depth_raw[i*512+j]/max_ir*256); // TODO:if I did wrong here
                        //img->at<char>(i,j) = int(ir_raw[i*512+j]/max_ir*256);
#else
                        img->at<cv::Vec3b>(i,j)[0] = *reinterpret_cast<int*>(&point.rgb) & 0xFF;       //  r
                        img->at<cv::Vec3b>(i,j)[1] = (*reinterpret_cast<int*>(&point.rgb) >> 8) & 0xFF;  // g
                        img->at<cv::Vec3b>(i,j)[2] = (*reinterpret_cast<int*>(&point.rgb) >> 16) & 0xFF;  //b
#endif
                        depth_img->at<ushort>(i,j) = ushort(depth_raw[i*512+j]);

                        if (isnan(point.x) || isnan(point.y) || isnan(point.z)) continue;
                        cloud->push_back(point);
                    }
                }
                gettimeofday(&tt2, NULL);
                printf("Time used %d for generate point cloud = %lf\n", current->id, gettime(tt1, tt2));

                int flag = imgdb.stage_sift(current->id);
                gettimeofday(&tt3, NULL);
                printf("Time used %d for image concatenate = %lf\n", current->id, gettime(tt2, tt3));

                if (!flag) imgdb.del();

                // if free, detect the edges
                imgdb.stage_edge(current->id, rgb, depth, &ir_params, &rgb_params);

                if (ImgBlurring) current->blurImg();
                if (ImgStore) current->storeImg();
                if (PCLVisualization) current->showPcl();
                if (PcdStore) current->storePcl();

#if USEg2o
                if (current->id == 0) {
                    g2o::VertexSE3* v = new g2o::VertexSE3();
                    v->setId( current->id );
                    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
                    v->setFixed( true ); //第一个顶点固定，不用优化
                    globalOptimizer.addVertex( v );
                }
                else {
                    // 向g2o中增加这个顶点与上一帧联系的边
                    // 顶点部分
                    // 顶点只需设定id即可
                    g2o::VertexSE3 *v = new g2o::VertexSE3();
                    v->setId( current->id );
                    v->setEstimate( Eigen::Isometry3d::Identity() );
                    globalOptimizer.addVertex(v);
                    // 边部分
                    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                    // 连接此边的两个顶点id
                    edge->vertices() [0] = globalOptimizer.vertex( current->id - 1 );
                    edge->vertices() [1] = globalOptimizer.vertex( current->id );

        edge->setRobustKernel( robustKernel ); // TODO: check as this sentence not ocurred before
                    // 信息矩阵
                    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
                    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
                    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
                    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
                    information(0,0) = information(1,1) = information(2,2) = 100;
                    information(3,3) = information(4,4) = information(5,5) = 100;
                    // 也可以将角度设大一些，表示对角度的估计更加准确
                    edge->setInformation( information );
                    // 边的估计即是pnp求解之结果
                    edge->setMeasurement( current->T );
                    // 将此边加入图中
                    globalOptimizer.addEdge(edge);

#if LOOP_DETECT // in the following two functions, matches would be abandoned if not match
                    imgdb.checkNearbyLoops(current->id, globalOptimizer);
                    imgdb.checkRandomLoops(current->id, globalOptimizer);
#endif

                }
#endif

                if (current->id % 20 == 0) {
                    if (current->id) {
#if USEg2o
                        // 优化所有边
                        printf("Optimizing pose graph, vertices: %lu\n", globalOptimizer.vertices().size());
#if SAVEg2o
                        char g2oname[20];
                        sprintf(g2oname, "%d_before.g2o", current->id);
                        globalOptimizer.save(g2oname);
#endif
                        globalOptimizer.initializeOptimization();
                        globalOptimizer.optimize( 100 ); //可以指定优化步数
                        gettimeofday(&tt4, NULL);
                        printf("Time used %d for g2o optimization = %lf\n", current->id, gettime(tt3, tt4));
#if SAVEg2o
                        sprintf(g2oname, "%d_after.g2o", current->id);
                        globalOptimizer.save(g2oname);
#endif
                        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>( globalOptimizer.vertex(current->id) );
                        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
                        current->globalT = imgdb.imglists[current->id-20]->globalT * pose;
                        // 以下是滤波
                        PointCloudT::Ptr tmp_cloud( new PointCloudT() );
                        /*voxel.setInputCloud( current->cloud );
                        voxel.filter( *tmp_cloud );
                        pass.setInputCloud( tmp_cloud );
                        pass.filter( *(current->cloud) );*/
                        // 把点云变换后加入全局地图中
                        pcl::transformPointCloud( *(current->cloud), *tmp_cloud, current->globalT.matrix() );

                        if (imgdb.global_viewer) {
                            char new_cloud_name[20];
                            sprintf(new_cloud_name, "global_cloud_%d", current->id);
                            imgdb.global_viewer->addPointCloud(tmp_cloud, new_cloud_name);
                            sprintf(new_cloud_name, "local_%d.pcd", current->id);
                            pcl::io::savePCDFile(new_cloud_name, *tmp_cloud);
                            sprintf(new_cloud_name, "lorigin_%d.pcd", current->id);
                            pcl::io::savePCDFile(new_cloud_name, *(current->cloud));
                            imgdb.global_viewer->spinOnce(1200);
                        }

                        *base += *tmp_cloud;
                        tmp_cloud->clear();
#if SAVE_GLOBAL
                        char global_cloud_name[20];
                        sprintf(global_cloud_name, "gcloud_%d.pcd", current->id);
                        pcl::io::savePCDFile(global_cloud_name, *base);
#endif 

                        imgdb.stage_concate(current->id);
                        gettimeofday(&tt5, NULL);
                        printf("Time used %d for img concatenate = %lf\n", current->id, gettime(tt4, tt5));

                        //globalOptimizer.clear();
#endif
                        printf("lin3 357\n");
                    }
                    else {
                        current->globalT = Eigen::Isometry3d::Identity();
                        
                        *base += *(current->cloud);
                        char new_cloud_name[20];
                        sprintf(new_cloud_name, "global_cloud_%d", current->id);
                        imgdb.global_viewer->addPointCloud(base, new_cloud_name);
                        imgdb.global_viewer->spinOnce(200);
                    }
                }

                ImageList *test = imgdb.getEdge();
                if (test) {
                    printf("find Edge of list %d\n", test->id);
                }
            }
            listener.release(frames);
        }
    }

    imgdb.unsetViewer();

    dev->stop();
    dev->close();
    
    return 0;
}

void* qby_event(void* args) {
    QBY *qby_param = (QBY*)args;


    libfreenect2::Registration *registration_qby = new libfreenect2::Registration(*(qby_param->ir_params), *(qby_param->color_params));
    libfreenect2::Frame undistorted_qby(512 , 424 , 4) , registered_qby(512 , 424 , 4);


    pcl::PointXYZRGB point;
    int pointflag = 0;
    cv::Mat rgb_img(1080, 1920, CV_8UC4, qby_param->rgb->data);
    imwrite("/home/chengkai/test/person2_2.jpg", rgb_img);

    cout << "followflag -------------------------------------------------" << followflag << endl;
    CURL *curl = curl_easy_init();
    CURLcode res = curl_global_init(CURL_GLOBAL_WIN32);
    HTTPRESULT = "";
    struct curl_httppost *formpost=NULL;
    struct curl_httppost *lastptr=NULL;
            //        struct curl_slist *headerlist=NULL;
            //        static const char buf[] = "Expect:";

            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "api_key",
                CURLFORM_COPYCONTENTS, "02391c3c527cb075436ef8bc7deb7c9f",
                CURLFORM_END);
            curl_formadd(&formpost,
                &lastptr,
                CURLFORM_COPYNAME, "api_secret",
                CURLFORM_COPYCONTENTS, "1hmI_vQhO-D4fyYkKe6LDIi22KRnU41i",
                CURLFORM_END);

            char* file_data = NULL;
            long file_size = 0;
            FILE* fp = fopen("/home/chengkai/test/person2_2.jpg","rb");
            cout << "reading file" << endl;
            if (fp)
            {
                cout<< "reading successfully" << endl;
                fseek(fp, 0, SEEK_END);
                file_size = ftell(fp);
                cout << file_size << endl;
                fseek(fp, 0, SEEK_SET);
                file_data = new char[file_size+1];
                fread(file_data,1,file_size,fp);
                fclose(fp);
            }


            curl_formadd(&formpost, &lastptr,
                CURLFORM_COPYNAME, "img",
                CURLFORM_BUFFER, "p2.jpg",
                CURLFORM_BUFFERPTR, file_data,
                CURLFORM_BUFFERLENGTH, file_size,
                CURLFORM_CONTENTTYPE, "image/jpeg",
                CURLFORM_END);

            if(curl) {
                cout << "start" << endl;
                curl_easy_setopt(curl, CURLOPT_URL, "http://apicn.faceplusplus.com/v2/detection/detect");
                curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);
                curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, true);
                curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &HTTPRESULT);
                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

                char error[1024];
                curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error);

                res = curl_easy_perform(curl);
                if(res != CURLE_OK) cout<<endl<<error<<endl;
            }

            curl_easy_cleanup(curl);
            curl_formfree(formpost);
	    cout << "debug qby_0" << endl;
		
         //   cout<<HTTPRESULT<<endl;

            FaceSet faceSet;
            faceSet.deserializeToObj(HTTPRESULT);
            faceSet.print();
            if(file_data != NULL)
                delete [] file_data;

             cout << "debug qby_1 " << faceSet.faceNumber << endl;
             for (int i = 0; i < faceSet.faceNumber; i++){
                 PersonSet personSet = GetAllPerson();
                 int flag = 0;
                 int isfollow = 0;
		 cout << "debug qby_2 " << personSet.personNumber << endl;
                 for (int j = 0; j < personSet.personNumber; j++){

                     cout << "qwertyy" << j << " result " << hasface(personSet.set[j],faceSet.set[i]) << " " << followflag << endl;
//                     if (hasface(personSet.set[j],faceSet.set[i])){
                        flag = 1;
                        if (followflag == 1){
                               cout << "followID----------------------------------" << followID << endl;
                               cout << "personSet.set--------------------------" <<  personSet.set[j].personID << endl;
                              /* for (int k = 0; k < 10; k++){
                                    if (followID[k] != personSet.set[j].personID[k]){
                                        isfollow = 1;
                                        break;
                                    }
                               }*/
                               cout << "isfollow---------------------------" << isfollow << endl;
                               if (isfollow == 0){
                                    float x = faceSet.set[i].centerX;
                                    float y = faceSet.set[i].centerY;
                                    float h = faceSet.set[i].height;
                                    float w = faceSet.set[i].width;
                                    int *mydata = (int*)qby_param->rgb->data;
                                    for (int p = 0; p < 1080; p++) {
                                        for (int q = 0; q < 1920; q++) {
//                                            if (p < 10.8 * (x + w/2) && p > 10.8 * (x - w /2) && q < 19.2 * (y + h/2) && q > 19.2 * (y - h/2)){
                                             if (p < 10.8 * (y + h/2) && p > 10.8 * (y - h /2) && q < 19.2 * (x + w/2) && q > 19.2 * (x - w/2)){
                                                mydata[p* 1920 + q] = mydata[p* 1920 + q] & (0xF0FF);
                                            }
                                        }
                                    }
                                    cv::Mat test_img(1080, 1920, CV_8UC4, qby_param->rgb->data);
                                    imwrite("/home/chengkai/test/test.jpg", test_img);
                                    
                                    uint32_t *rgb_raw = (uint32_t*)qby_param->rgb->data;
                                    float *depth_raw = (float*)qby_param->depth->data;
                                    registration_qby->apply(qby_param->rgb , qby_param->depth , &undistorted_qby , &registered_qby);
                                    float point_depth;
                                    for (int i = 0; i < 424; i++) {
                                        int loopflag = 0;
                                        for (int j = 0; j < 512; j++) {
                                            if (i < 4.24 * (y + h/2) && i > 4.24 * (y - h /2) && j < 5.12 * (x + w/2) && j > 5.12 * (x - w/2)){
                                                registration_qby->getPointXYZRGB(&undistorted_qby, &registered_qby, i, j,
                                                    point.x, point.y, point.z, point.rgb);
                                                uint32_t color =  (*reinterpret_cast<int*>(&point.rgb) >> 16) & 0xFF;
						cout << "color" << color << endl;
                                                if (color == 0){
                                                    point_depth = depth_raw[i*512+j];
						    cout << "point.x----->" << point.x << "point.z------>" << point.z << endl;
                                                    loopflag = 1;
                                                    pointflag = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        if (loopflag == 1){
                                             break;
                                        }
                                    }

  //                             }
                        }
                        break;
                     }
                 }
                 if (flag == 0){             //如果不在已知集合中，需要创建人物
                     createPerson(faceSet.set[i]);
/* 				float x = faceSet.set[i].centerX;
                                    float y = faceSet.set[i].centerY;
                                    float h = faceSet.set[i].height;
                                    float w = faceSet.set[i].width;
                                    int *mydata = (int*)qby_param->rgb->data;
                                    for (int p = 0; p < 1080; p++) {
                                        for (int q = 0; q < 1920; q++) {
//                                            if (p < 10.8 * (x + w/2) && p > 10.8 * (x - w /2) && q < 19.2 * (y + h/2) && q > 19.2 * (y - h/2)){
                                             if (p < 10.8 * (y + h/2) && p > 10.8 * (y - h /2) && q < 19.2 * (x + w/2) && q > 19.2 * (x - w/2)){
                                                mydata[p* 1920 + q] = mydata[p* 1920 + q] & (0xF0FF);
                                            }
                                        }
                                    }
                                    cv::Mat test_img(1080, 1920, CV_8UC4, qby_param->rgb->data);
                                    imwrite("/home/chengkai/test/test.jpg", test_img);
                                    
                                    uint32_t *rgb_raw = (uint32_t*)qby_param->rgb->data;
                                    float *depth_raw = (float*)qby_param->depth->data;
                                    registration_qby->apply(qby_param->rgb , qby_param->depth , &undistorted_qby , &registered_qby);
                                    float point_depth;
                                    for (int i = 0; i < 424; i++) {
                                        int loopflag = 0;
                                        for (int j = 0; j < 512; j++) {
                                            if (i < 4.24 * (y + h/2) && i > 4.24 * (y - h /2) && j < 5.12 * (x + w/2) && j > 5.12 * (x - w/2)){
                                                registration_qby->getPointXYZRGB(&undistorted_qby, &registered_qby, i, j,
                                                    point.x, point.y, point.z, point.rgb);
                                                uint32_t color =  (*reinterpret_cast<int*>(&point.rgb) >> 16) & 0xFF;
						cout << "color" << color << endl;
                                                if (color == 0){
                                                    point_depth = depth_raw[i*512+j];
						    cout << "point.x----->" << point.x << "point.z------>" << point.z << endl;
                                                    loopflag = 1;
                                                    pointflag = 1;
                                                    break;
                                                }
                                            }
                                        }
                                        if (loopflag == 1){
                                             break;
                                        }
                                    }*/

                 }
             }

        if (pointflag == 1) {
            qby_param->current->person_point = point;
        }
        else qby_param->current->person_point = pcl::PointXYZRGB(0,0,0);

        qby_param->imgdb->qby_thread_in_use = false;
        if (!qby_param->imgdb->edge_thread_in_use) qby_param->imgdb->edge_thread_item = qby_param->current;

	printf("REOPLE End %d\n", qby_param->current->id);

        return NULL;
}

