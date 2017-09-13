//ds THIS CODE WAS CREATED BASED ON: http://kitti.is.tue.mpg.de/kitti/devkit_odometry.zip
//ds minimally modified to avoid C++11 warnings and provided a brief result dump to stdout

#include <iostream>
#include <limits>
#include <fstream>
#include <Eigen/Geometry>

#include <srrg_system_utils/system_utils.h>

//ds readability
typedef Eigen::Matrix<float, 4, 4> Matrix4;

// static parameter
// float lengths[] = {5,10,50,100,150,200,250,300,350,400};
float lengths[] = {100,200,300,400,500,600,700,800};
int32_t num_lengths = 8;

struct errors {
    int32_t first_frame;
    float   r_err;
    float   t_err;
    float   len;
    float   speed;
    errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
        first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

std::vector<Matrix4> loadPoses(std::string file_name) {
    std::vector<Matrix4> poses;

    //ds going modern
    std::ifstream pose_file(file_name, std::ifstream::in);

    //ds grab a line from the ground truth
    std::string buffer_line;
    while (std::getline(pose_file, buffer_line)) {

        //ds get it to a std::stringstream
        std::istringstream buffer_stream(buffer_line);

        //ds information fields (KITTI format)
        Matrix4 pose(Matrix4::Identity());
        for (uint8_t u = 0; u < 3; ++u) {
            for (uint8_t v = 0; v < 4; ++v) {
                buffer_stream >> pose(u,v);
            }
        }
        poses.push_back(pose);
    }

    pose_file.close();
    return poses;
}

std::vector<float> trajectoryDistances (std::vector<Matrix4> &poses) {
    std::vector<float> dist;
    dist.push_back(0);
    for (std::size_t i=1; i<poses.size(); i++) {
        Matrix4 P1 = poses[i-1];
        Matrix4 P2 = poses[i];
        float dx = P1(0,3)-P2(0,3);
        float dy = P1(1,3)-P2(1,3);
        float dz = P1(2,3)-P2(2,3);
        dist.push_back(dist[i-1]+std::sqrt(dx*dx+dy*dy+dz*dz));
    }
    return dist;
}

int32_t lastFrameFromSegmentLength(std::vector<float> &dist,int32_t first_frame,float len) {
    for (std::size_t i=first_frame; i<dist.size(); i++)
        if (dist[i]>dist[first_frame]+len)
            return i;
    return -1;
}

inline float rotationError(Matrix4 &pose_error) {
    float a = pose_error(0,0);
    float b = pose_error(1,1);
    float c = pose_error(2,2);
    float d = 0.5*(a+b+c-1.0);
    return std::acos(std::max(std::min(d,1.0f),-1.0f));
}

inline float translationError(Matrix4 &pose_error) {
    float dx = pose_error(0,3);
    float dy = pose_error(1,3);
    float dz = pose_error(2,3);
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

std::vector<errors> calcSequenceErrors (std::vector<Matrix4> &poses_gt,std::vector<Matrix4> &poses_result) {

    // error std::vector
    std::vector<errors> err;

    // parameters
    int32_t step_size = 10; // every second

    // pre-compute distances (from ground truth as reference)
    std::vector<float> dist = trajectoryDistances(poses_gt);

    // for all start positions do
    for (std::size_t first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {

        // for all segment lengths do
        for (int32_t i=0; i<num_lengths; i++) {

            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len);

            // continue, if sequence not long enough
            if (last_frame==-1)
                continue;

            // compute rotational and translational errors
            Matrix4 pose_delta_gt     = poses_gt[first_frame].inverse()*poses_gt[last_frame];
            Matrix4 pose_delta_result = poses_result[first_frame].inverse()*poses_result[last_frame];
            Matrix4 pose_error        = pose_delta_result.inverse()*pose_delta_gt;
            float r_err = rotationError(pose_error);
            float t_err = translationError(pose_error);

            // compute speed
            float num_frames = (float)(last_frame-first_frame+1);
            float speed = len/(0.1*num_frames);

            // write to file
            err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
        }
    }

    // return error std::vector
    return err;
}

void saveSequenceErrors (std::vector<errors> &err,std::string file_name) {

    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(),"w");

    // write to file
    for (std::vector<errors>::iterator it=err.begin(); it!=err.end(); it++)
        fprintf(fp,"%d %f %f %f %f\n",it->first_frame,it->r_err,it->t_err,it->len,it->speed);

    // close file
    fclose(fp);
}

void savePathPlot (std::vector<Matrix4> &poses_gt,std::vector<Matrix4> &poses_result,std::string file_name) {

    // parameters
    int32_t step_size = 3;

    // open file
    FILE *fp = fopen(file_name.c_str(),"w");

    // save x/z coordinates of all frames to file
    for (std::size_t i=0; i<poses_gt.size(); i+=step_size)
        fprintf(fp,"%f %f %f %f\n",poses_gt[i](0,3),poses_gt[i](2,3),
                poses_result[i](0,3),poses_result[i](2,3));

                // close file
                fclose(fp);
}

std::vector<int32_t> computeRoi (std::vector<Matrix4> &poses_gt,std::vector<Matrix4> &poses_result) {

    float x_min = std::numeric_limits<int32_t>::max();
    float x_max = std::numeric_limits<int32_t>::min();
    float z_min = std::numeric_limits<int32_t>::max();
    float z_max = std::numeric_limits<int32_t>::min();

    for (std::vector<Matrix4>::iterator it=poses_gt.begin(); it!=poses_gt.end(); it++) {
        float x = (*it)(0,3);
        float z = (*it)(2,3);
        if (x<x_min) x_min = x; if (x>x_max) x_max = x;
        if (z<z_min) z_min = z; if (z>z_max) z_max = z;
    }

    for (std::vector<Matrix4>::iterator it=poses_result.begin(); it!=poses_result.end(); it++) {
        float x = (*it)(0,3);
        float z = (*it)(2,3);
        if (x<x_min) x_min = x; if (x>x_max) x_max = x;
        if (z<z_min) z_min = z; if (z>z_max) z_max = z;
    }

    float dx = 1.1*(x_max-x_min);
    float dz = 1.1*(z_max-z_min);
    float mx = 0.5*(x_max+x_min);
    float mz = 0.5*(z_max+z_min);
    float r  = 0.5*std::max(dx,dz);

    std::vector<int32_t> roi;
    roi.push_back((int32_t)(mx-r));
    roi.push_back((int32_t)(mx+r));
    roi.push_back((int32_t)(mz-r));
    roi.push_back((int32_t)(mz+r));
    return roi;
}

int32_t plotPathPlot (std::string dir,std::vector<int32_t> &roi,int32_t idx) {

    // gnuplot file name
    char command[1024];
    char file_name[256];
    sprintf(file_name,"%02d.gp",idx);
    std::string full_name = dir + "/" + file_name;

    //ds system calls
    int32_t result = 0;

    // create png + eps
    for (int32_t i=0; i<2; i++) {

        // open file
        FILE *fp = fopen(full_name.c_str(),"w");

        // save gnuplot instructions
        if (i==0) {
            fprintf(fp,"set term png size 900,900\n");
            fprintf(fp,"set output \"%02d.png\"\n",idx);
        } else {
            fprintf(fp,"set term postscript eps enhanced color\n");
            fprintf(fp,"set output \"%02d.eps\"\n",idx);
        }

        fprintf(fp,"set size ratio -1\n");
        fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
        fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
        fprintf(fp,"set xlabel \"x [m]\"\n");
        fprintf(fp,"set ylabel \"z [m]\"\n");
        fprintf(fp,"plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,",idx);
        fprintf(fp,"\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' w lines,",idx);
        fprintf(fp,"\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",idx);

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
        result = system(command);
    }

    // create pdf and crop
    sprintf(command,"cd %s; ps2pdf %02d.eps %02d_large.pdf",dir.c_str(),idx,idx);
    result = system(command);
    sprintf(command,"cd %s; pdfcrop %02d_large.pdf %02d.pdf",dir.c_str(),idx,idx);
    result = system(command);
    sprintf(command,"cd %s; rm %02d_large.pdf",dir.c_str(),idx);
    result = system(command);
    return result;
}

void saveErrorPlots(std::vector<errors> &seq_err,std::string plot_error_dir,const char* prefix) {

    // file names
    char file_name_tl[1024]; sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix);
    char file_name_rl[1024]; sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix);
    char file_name_ts[1024]; sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix);
    char file_name_rs[1024]; sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix);

    // open files
    FILE *fp_tl = fopen(file_name_tl,"w");
    FILE *fp_rl = fopen(file_name_rl,"w");
    FILE *fp_ts = fopen(file_name_ts,"w");
    FILE *fp_rs = fopen(file_name_rs,"w");

    // for each segment length do
    std::cerr << "---------------------------- ERROR STATISTICS ----------------------------" << std::endl;
    float total_error_rotation    = 0;
    float total_error_translation = 0;
    uint32_t number_of_lengths = 0;
    for (int32_t i=0; i<num_lengths; i++) {

        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (std::vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->len-lengths[i])<1.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num>2.5) {
            fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
            fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
            total_error_rotation    += r_err/num*(180/M_PI)*100;
            total_error_translation += t_err/num*100;
            ++number_of_lengths;

            //ds info
            std::printf("length: %f error rotation (deg/100m): %9.6f error translation (%%): %9.6f\n", lengths[i], r_err/num*(180/M_PI)*100, t_err/num*100);
        }
    }
    std::cerr << "---------------------------- ---------------- ----------------------------" << std::endl;
    std::printf("average error rotation (deg/100m): %9.6f error translation (%%): %9.6f\n", total_error_rotation/number_of_lengths, total_error_translation/number_of_lengths);
    std::cerr << "---------------------------- ---------------- ----------------------------" << std::endl;

    // for each driving speed do (in m/s)
    for (float speed=2; speed<25; speed+=2) {

        float t_err = 0;
        float r_err = 0;
        float num   = 0;

        // for all errors do
        for (std::vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
            if (fabs(it->speed-speed)<2.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num>2.5) {
            fprintf(fp_ts,"%f %f\n",speed,t_err/num);
            fprintf(fp_rs,"%f %f\n",speed,r_err/num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

int32_t plotErrorPlots (std::string dir,const char* prefix) {

    char command[1024];

    //ds system calls
    int32_t result = 0;

    // for all four error plots do
    for (int32_t i=0; i<4; i++) {

        // create suffix
        char suffix[16];
        switch (i) {
        case 0: sprintf(suffix,"tl"); break;
        case 1: sprintf(suffix,"rl"); break;
        case 2: sprintf(suffix,"ts"); break;
        case 3: sprintf(suffix,"rs"); break;
        }

        // gnuplot file name
        char file_name[1024]; char full_name[1024];
        sprintf(file_name,"%s_%s.gp",prefix,suffix);
        sprintf(full_name,"%s/%s",dir.c_str(),file_name);

        // create png + eps
        for (int32_t j=0; j<2; j++) {

            // open file
            FILE *fp = fopen(full_name,"w");

            // save gnuplot instructions
            if (j==0) {
                fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp,"set output \"%s_%s.png\"\n",prefix,suffix);
            } else {
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%s_%s.eps\"\n",prefix,suffix);
            }

            // start plot at 0
            fprintf(fp,"set size ratio 0.5\n");
            fprintf(fp,"set yrange [0:*]\n");

            // x label
            if (i<=1) fprintf(fp,"set xlabel \"Path Length [m]\"\n");
            else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");

            // y label
            if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
            else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");

            // plot error curve
            fprintf(fp,"plot \"%s_%s.txt\" using ",prefix,suffix);
            switch (i) {
            case 0: fprintf(fp,"1:($2*100) title 'Translation Error'"); break;
            case 1: fprintf(fp,"1:($2*57.3) title 'Rotation Error'"); break;
            case 2: fprintf(fp,"($1*3.6):($2*100) title 'Translation Error'"); break;
            case 3: fprintf(fp,"($1*3.6):($2*57.3) title 'Rotation Error'"); break;
            }
            fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
            result = system(command);
        }

        // create pdf and crop
        sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
        result = system(command);
        sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
        result = system(command);
        sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix,suffix);
        result = system(command);
    }

    return result;
}

void saveStats (std::vector<errors> err,std::string dir) {

    float t_err = 0;
    float r_err = 0;

    // for all errors do => compute sum of t_err, r_err
    for (std::vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
        t_err += it->t_err;
        r_err += it->r_err;
    }

    // open file
    FILE *fp = fopen((dir + "/stats.txt").c_str(),"w");

    // save errors
    float num = err.size();
    fprintf(fp,"%f %f\n",t_err/num,r_err/num);

    // close file
    fclose(fp);
}

bool eval (const std::string& file_trajectory_test_, const std::string& file_trajectory_ground_truth_, const std::string& file_sequence_) {

    // ground truth and result directories
    std::string result_dir     = "results";
    std::string error_dir      = result_dir + "/errors";
    std::string plot_path_dir  = result_dir + "/plot_path";
    std::string plot_error_dir = result_dir + "/plot_error";

    int32_t result = 0;

    // create output directories
    result = system(("mkdir " + result_dir).c_str());
    result = system(("mkdir " + error_dir).c_str());
    result = system(("mkdir " + plot_path_dir).c_str());
    result = system(("mkdir " + plot_error_dir).c_str());
    if (result < 0) {
        printf("system io error\n");
        return false;
    }

    // read ground truth and result poses
    std::vector<Matrix4> poses_gt     = loadPoses(file_trajectory_ground_truth_);
    std::vector<Matrix4> poses_result = loadPoses(file_trajectory_test_);

    //ds parse sequence number
    const uint32_t sequence_number = std::stoi(file_sequence_.substr(0, 2));

    // plot status
    printf("Processing: %s (sequence: %i), poses: %lu/%lu\n",file_sequence_.c_str(),sequence_number,poses_result.size(),poses_gt.size());

    // check for errors
    if (poses_gt.size()==0 || poses_result.size()!=poses_gt.size()) {
        printf("ERROR: Couldn't read (all) poses of: %s\n", file_sequence_.c_str());
        return false;
    }

    // compute sequence errors
    std::vector<errors> seq_err = calcSequenceErrors(poses_gt,poses_result);
    saveSequenceErrors(seq_err,error_dir + "/" + file_sequence_);

    // for first half => plot trajectory and compute individual stats
    // save + plot bird's eye view trajectories
    savePathPlot(poses_gt,poses_result,plot_path_dir + "/" + file_sequence_);
    std::vector<int32_t> roi = computeRoi(poses_gt,poses_result);
    plotPathPlot(plot_path_dir,roi,sequence_number);

    // save + plot individual errors
    saveErrorPlots(seq_err,plot_error_dir,file_sequence_.substr(0, 2).c_str());
    plotErrorPlots(plot_error_dir,file_sequence_.substr(0, 2).c_str());

    // success
    return true;
}


const char* banner[] = {
    "\n\nUsage: ./srrg_kitti_evaluate_odometry_app -gt <string> -odom <string> -seq <string>"
    "\n",
    "Example: ./srrg_kitti_evaluate_odometry_app -gt 00.txt -odom tracker_output.txt -seq 00.txt",
    "Options:\n", "------------------------------------------\n",
    "-gt <string>                path to ground_truth file",
    "-odom <string>              path to tracker odometry path",
    "-seq <string>               sequence number, e.g. '00.txt' if not set, then seq=gt",
    "-h                          this help\n", 0};

int32_t main (int32_t argc, char** argv) {

    std::string file_tracked_odometry = "";
    std::string file_ground_truth     = "";
    std::string file_sequence         = "";

    int c = 1;
    while(c < argc) {
        if (!strcmp(argv[c], "-h")) {
            srrg_core::printBanner(banner);
            return 1;
        } else if (!strcmp(argv[c], "-gt")) {
            c++;
            file_ground_truth = argv[c];
        } else if (!strcmp(argv[c], "-odom")) {
            c++;
            file_tracked_odometry = argv[c];
        } else if (!strcmp(argv[c], "-seq")) {
            c++;
            file_sequence = argv[c];
        }
        c++;
    }

    if(file_sequence.empty())
        file_sequence = file_ground_truth;


    // run evaluation
    return eval(file_tracked_odometry, file_ground_truth, file_sequence);
}
