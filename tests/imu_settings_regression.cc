#include "System.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace ORB_SLAM3;
namespace fs=std::filesystem;
static void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
static std::string read(const fs::path& file) { std::ifstream in(file);return {std::istreambuf_iterator<char>(in),{}}; }
static std::string setYaml(std::string text,const std::string& key,const std::string& value)
{
    std::istringstream in(text);std::ostringstream out;std::string line;bool found=false;
    while(std::getline(in,line)) {
        if(line.rfind(key+":",0)==0) {line=key+": "+value;found=true;}
        out<<line<<"\n";
    }
    if(!found) out<<key<<": "<<value<<"\n";
    return out.str();
}
class TrackingProbe : public Tracking {
public:
    using Tracking::Tracking;
    int ResetWindow() const {return mnFramesToResetIMU;}
    float Frequency() const {return mImuFreq;}
    bool InsertLost() const {return mInsertKFsLost;}
};
class MappingProbe : public LocalMapping {
public:
    using LocalMapping::LocalMapping;
    float FirstTime() const {return mVIBA1Time;}
    float SecondTime() const {return mVIBA2Time;}
    bool TryEmptyInit() {return InitializeIMU();}
};

int main(int argc,char** argv)
{
    if(argc!=2) return 2;
    const fs::path root=argv[1];
    const fs::path temp=fs::temp_directory_path()/("orb-imu-settings-"+std::to_string(getpid()));
    fs::create_directories(temp);
    struct Cleanup {fs::path path;~Cleanup(){fs::remove_all(path);}} cleanup{temp};
    const auto modern=read(root/"Examples/Stereo-Inertial/RealSense_D435i.yaml");
    const auto legacy=read(root/"Examples_old/Stereo-Inertial/RealSense_D435i.yaml");
    require(!modern.empty() && !legacy.empty(),"Missing fixtures");
    for(bool old : {false,true}) for(bool fast : {false,true}) {
        std::string yaml=old?legacy:modern;
        yaml=setYaml(yaml,"IMU.fastInit",fast?"1":"0");
        yaml=setYaml(yaml,"IMU.VIBA1Time","3");yaml=setYaml(yaml,"IMU.VIBA2Time","10.0");
        yaml=setYaml(yaml,"IMU.Frequency",old?"200.0":"200");
        yaml=setYaml(yaml,"Camera.fps","30.0");
        const auto file=temp/(old?"legacy.yaml":"modern.yaml");std::ofstream(file)<<yaml;
        cv::FileStorage input(file.string(),cv::FileStorage::READ);
        const auto config=ImuSettings::Read(input);
        std::unique_ptr<Settings> settings;
        if(!old) settings=std::make_unique<Settings>(file.string(),System::IMU_STEREO);
        ORBVocabulary voc;KeyFrameDatabase db(voc);Atlas atlas(0);atlas.SetInertialSensor();
        FrameDrawer drawer(&atlas);
        MapDrawer mapDrawer(&atlas,file.string(),settings.get());
        TrackingProbe tracker(nullptr,&voc,&drawer,&mapDrawer,&atlas,&db,file.string(),System::IMU_STEREO,settings.get());
        require(tracker.mFastInit==fast,"fastInit not applied to Tracking");
        require(tracker.ResetWindow()==30,"IMU recovery frame window not initialized");
        require(tracker.Frequency()==200,"Integer/real IMU frequency parsing differs");
        require(!tracker.InsertLost(),"InsertKFsWhenLost not applied");
        MappingProbe mapper(nullptr,&atlas,false,true,"",config.viba1Time,config.viba2Time);
        require(mapper.FirstTime()==3 && mapper.SecondTime()==10,"VIBA times not propagated to mapper");
        require(!mapper.TryEmptyInit(),"Empty map falsely completed initialization");
        require(!atlas.GetCurrentMap()->GetIniertialBA1() && !atlas.GetCurrentMap()->GetIniertialBA2(),
                "Deferred initialization marked VIBA complete");
    }
    const std::string minimal="%YAML:1.0\nIMU.Frequency: 200\nIMU.NoiseGyro: 0.001\nIMU.NoiseAcc: 0.01\nIMU.GyroWalk: 0\nIMU.AccWalk: 0\n"
        "IMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: d\n   data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n";
    auto parse=[](const std::string& yaml){cv::FileStorage f(yaml,cv::FileStorage::READ|cv::FileStorage::MEMORY);return ImuSettings::Read(f);};
    const auto defaults=parse(minimal);
    require(!defaults.fastInit && defaults.viba1Time==5 && defaults.viba2Time==15 && defaults.insertKFsWhenLost,
            "Absent optional IMU parameters changed default behavior");
    require(defaults.Tbc.type()==CV_32F,"Double extrinsics not converted safely");
    require(parse(setYaml(setYaml(minimal,"IMU.VIBA1Time","55"),"IMU.VIBA2Time","65")).viba2Time==65,
            "Long VIBA thresholds must be supported");
    for(const auto& bad : std::vector<std::pair<std::string,std::string>>{
        {"IMU.fastInit","2"},{"IMU.fastInit","\"off\""},{"IMU.Frequency","0"},
        {"IMU.Frequency",".Nan"},{"IMU.NoiseAcc","-1"},{"IMU.GyroWalk","-1"},
        {"IMU.VIBA1Time","0"},{"IMU.VIBA1Time","20"},{"IMU.VIBA2Time","5"},
        {"IMU.VIBA2Time","-1"},{"IMU.VIBA2Time",".Inf"},{"IMU.VIBA2Time","\"10\""}}) {
        bool rejected=false;try {parse(setYaml(minimal,bad.first,bad.second));} catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Invalid IMU parameter accepted");
    }
    bool conflict=false;
    try {parse(minimal+"IMU.InsertKFsWhenLost: 0\nInsertKFsWhenLost: 1\n");}
    catch(const std::invalid_argument&){conflict=true;}
    require(conflict,"Conflicting legacy/current keys silently accepted");
    // A right camera must use its own calibration, for both camera models.
    for(const std::string model : {"PinHole","KannalaBrandt8"}) {
        auto yaml=setYaml(modern,"Camera.type","\""+model+"\"");
        for(int camera=1;camera<=2;++camera) {
            const std::string prefix="Camera"+std::to_string(camera)+".";
            for(const auto& kv : std::vector<std::pair<std::string,std::string>>{{"fx","400"},{"fy","400"},{"cx","320"},{"cy","240"},
                {"k1",camera==1?"0.01":"0.02"},{"k2",camera==1?"0.001":"0.002"},{"p1","0"},{"p2","0"},
                {"k3","0"},{"k4","0"},{"overlappingBegin","0"},{"overlappingEnd","640"}})
                yaml=setYaml(yaml,prefix+kv.first,kv.second);
        }
        yaml+="Stereo.T_c1_c2: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [1,0,0,0.05, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n";
        const auto path=temp/(model+".yaml");std::ofstream(path)<<yaml;
        Settings stereo(path.string(),System::IMU_STEREO);
        if(model=="PinHole")
            require(std::abs(stereo.camera2DistortionCoef().at<float>(0)-0.02f)<1e-6f,"Right pinhole distortion copied left coefficients");
        else
            require(std::abs(stereo.camera2()->getParameter(4)-0.02f)<1e-6f,"Right fisheye distortion loaded Camera1 keys");
    }
    // Pure stereo must not allocate/read an unconfigured IMU calibration.
    const auto visual=temp/"visual.yaml";std::ofstream(visual)<<modern;
    Settings visualSettings(visual.string(),System::STEREO);
    ORBVocabulary voc;KeyFrameDatabase db(voc);Atlas atlas(0);FrameDrawer drawer(&atlas);
    MapDrawer mapDrawer(&atlas,visual.string(),&visualSettings);
    TrackingProbe tracker(nullptr,&voc,&drawer,&mapDrawer,&atlas,&db,visual.string(),System::STEREO,&visualSettings);
    require(tracker.ResetWindow()==0,"Visual-only tracking has an IMU reset window");
    std::cout<<"IMU_SETTINGS_PASS: both loaders, runtime flags/window/times, defaults, numeric types, invalid inputs, visual-only path"<<std::endl;
}
