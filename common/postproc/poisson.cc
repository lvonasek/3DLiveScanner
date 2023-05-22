#include <data/file3d.h>
#include <data/mesh.h>
#include <postproc/poisson.h>

extern int poisson_main( int argc , char* argv[] );

namespace oc {
    void Poisson::Process(std::string filename) {
        int kSubdivisionSize = 20000;
        std::string after = filename + "-after_poisson.ply";
        std::string before = filename + "-before_poisson.ply";

        //convert to ply
        {
            std::vector<Mesh> data;
#ifdef ANDROID
            File3d(filename, false).ReadModel(kSubdivisionSize, data);
#else
            File3d(filename + "-copy.obj", false).ReadModel(kSubdivisionSize, data);
#endif
            File3d(before, true).WriteModel(data);
        }

        //poisson reconstruction
        std::vector<std::string> argv;
        argv.push_back("--in");
        argv.push_back(before);
        argv.push_back("--out");
        argv.push_back(after);
        std::vector<const char *> av;
        av.push_back(0);
        for (std::vector<std::string>::const_iterator i = argv.begin(); i != argv.end(); ++i)
            av.push_back(i->c_str());
        poisson_main((int) av.size(), (char **) &av[0]);

        //convert to obj
        std::vector<Mesh> data;
        File3d(after, false).ReadModel(kSubdivisionSize, data);
        File3d(filename, true).WriteModel(data);
    }
}
