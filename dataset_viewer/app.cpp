#include <data/dataset.h>
#include <data/file3d.h>
#include <gl/glsl.h>
#include <GL/freeglut.h>

//shader
const char* kTextureShader[] = {R"glsl(
    uniform mat4 u_MVP;
    attribute vec3 v_vertex;
    attribute vec2 v_coord;
    varying vec2 v_UV;

    void main() {
      v_UV = vec2(v_coord.x, 1.0 - v_coord.y);
      gl_Position = u_MVP * vec4(v_vertex, 1.0);
    })glsl",

    R"glsl(
    uniform sampler2D u_texture;
    varying vec2 v_UV;

    void main() {
      gl_FragColor = texture2D(u_texture, v_UV);
    })glsl"
};

bool mouseActive = true;
float mouseX;          ///< Last mouse X
float mouseY;          ///< Last mouse X
float pitch;           ///< Camera pitch
float yaw;             ///< Camera yaw
glm::vec3 camera;      ///< Camera position
glm::mat4 proj;        ///< Camera projection
glm::mat4 view;        ///< Camera view
bool keys[4];          ///< State of keyboard
glm::ivec2 resolution; ///< Screen resolution
int poseCount;         ///< Pose count
int poseIndex;         ///< Pose index
oc::Dataset* dataset;  ///< Path to the dataset
oc::GLSL* shader;      ///< Shader;
bool ready;            ///< Loading status

//render data
std::map<int, int> loaded;
std::vector<oc::Mesh> model;
std::map<std::string, GLuint> texture;

#define KEY_UP 'w'
#define KEY_DOWN 's'
#define KEY_LEFT 'a'
#define KEY_RIGHT 'd'

void clear()
{
    for (std::pair<std::string, GLuint> i : texture) {
        glDeleteTextures(1, &i.second);
    }
    loaded.clear();
    model.clear();
    texture.clear();
}

void loadFrame()
{
    if (loaded.find(poseIndex) == loaded.end())
    {
        int size = 0;
        FILE* file = fopen(dataset->GetFileName(poseIndex, ".pcl").c_str(), "rb");
        fread(&size, sizeof(int), 1, file);
        Tango3DR_Vector4* points = new Tango3DR_Vector4[size];
        fread(points, sizeof(Tango3DR_Vector4), size, file);
        fclose(file);

        glm::mat4 depth_mat = dataset->ReadPose(poseIndex)[oc::COLOR_CAMERA];
        glm::mat4 viewproj_mat = dataset->ReadPose(poseIndex)[oc::SCREEN_CAMERA];

        oc::Mesh pcl;
        for (unsigned int i = 0; i < size; i++) {
            glm::vec4 v = depth_mat * glm::vec4(points[i][0], points[i][1], points[i][2], 1.0f);
            glm::vec4 t = viewproj_mat * glm::vec4(v.x, v.y, v.z, 1.0f);
            t /= fabs(t.w);
            t = t * 0.5f + 0.5f;
            pcl.uv.push_back(glm::vec2(t.x, t.y));
            pcl.vertices.push_back(glm::vec3(v.x, v.y, v.z));
        }
        pcl.image = new oc::Image(dataset->GetFileName(poseIndex, ".jpg"));
        pcl.imageOwner = true;

        delete[] points;
        loaded[poseIndex] = model.size();
        model.push_back(pcl);
    }
}

/**
 * @brief display updates display
 */
void display(void)
{
    /// set buffers
    glClearColor(0, 0, 0.5f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, resolution.x, resolution.y);
    glEnable(GL_DEPTH_TEST);

    /// set view
    float aspect = resolution.x / (float)resolution.y;
    glm::vec3 eye = glm::vec3(0, 0, 0);
    glm::vec3 center = glm::vec3(sin(yaw) * fabs(cos(pitch)), -sin(pitch), -cos(yaw) * fabs(cos(pitch)));
    glm::vec3 up = glm::vec3(0, 1, 0);
    eye += camera;
    center += camera;
    proj = glm::perspective(glm::radians(60.0f), aspect, 0.01f, 500.0f);
    view = glm::lookAt(eye, center, up);

    /// render model
    int index = 0;
    shader->Bind();
    for (oc::Mesh& m : model) {
        if (!m.vertices.empty()) {
            if (texture.find(m.image->GetName()) == texture.end()) {
                texture[m.image->GetName()] = oc::GLSL::Image2GLTexture(m.image);
            }
            glBindTexture(GL_TEXTURE_2D, texture[m.image->GetName()]);
            shader->UniformMatrix("u_MVP", glm::value_ptr(proj * view));
            shader->Attrib(&m.vertices[0].x, 0, &m.uv[0].x, 0);
            glDrawArrays(GL_POINTS, 0, (GLsizei) m.vertices.size());
        }
    }
    shader->Unbind();

    /// check if there is an error
    int i = glGetError();
    if (i != 0)
        printf("GL_ERROR %d\n", i);

    /// text info
    unsigned char text[1024];
    if (!ready)
    {
        sprintf((char*)text, "Loading... still %d frames to load", poseCount - poseIndex);
    }
    else if (loaded.find(poseIndex) != loaded.end())
    {
        sprintf((char*)text, "Currently selected frame: %d", poseIndex);
    }
    else
    {
        sprintf((char*)text, "Currently selected frame: none");
    }

    glColor3f(1, 0, 0);
    glRasterPos2f(-0.99f, 0.95f);
    glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, text);

    /// finish rendering
    glutSwapBuffers();
}

/**
 * @brief idle is non-graphical thread and it is called automatically by GLUT
 * @param v is time information
 */
void idle(int v)
{
    glm::mat4 direction = view;
    direction[3][0] = 0;
    direction[3][1] = 0;
    direction[3][2] = 0;
    glm::vec4 side = glm::vec4(0, 0, 0, 1);
    if (keys[0] && keys[1])
      side = glm::vec4(0, 0, 0, 1);
    else if (keys[0])
      side = glm::vec4(-1, 0, 0, 1) * direction;
    else if (keys[1])
      side = glm::vec4(1, 0, 0, 1) * direction;
    side /= glm::abs(side.w) * 10.0f;
    camera.x += side.x;
    camera.y += side.y;
    camera.z += side.z;
    glm::vec4 forward = glm::vec4(0, 0, 0, 1);
    if (keys[2] && keys[3])
      forward = glm::vec4(0, 0, 0, 1);
    else if (keys[2])
      forward = glm::vec4(0, 0, -1, 1) * direction;
    else if (keys[3])
      forward = glm::vec4(0, 0, 1, 1) * direction;
    forward /= glm::abs(forward.w) * 10.0f;
    camera.x += forward.x;
    camera.y += forward.y;
    camera.z += forward.z;

    if (ready && mouseActive) {
        pitch += (mouseY - resolution.y / 2) / (float)resolution.y;
        yaw += (mouseX - resolution.x / 2) / (float)resolution.x;
        glutWarpPointer(resolution.x / 2, resolution.y / 2);
    }
    glutSetCursor(mouseActive ? GLUT_CURSOR_NONE : GLUT_CURSOR_LEFT_ARROW);

    if (pitch < -1.57f)
        pitch = -1.57f;
    if (pitch > 1.57f)
        pitch = 1.57f;

    /// load frames
    if (!ready)
    {
        poseIndex++;
        if (poseIndex == poseCount)
        {
            poseIndex = 0;
            ready = true;
        } else {
            loadFrame();
        }
    }

    /// call update
    glutPostRedisplay();
    glutTimerFunc(50,idle,0);
}

/**
 * @brief keyboardDown is called when key pressed
 * @param key is key code
 * @param x is cursor position x
 * @param y is cursor position y
 */
void keyboardDown(unsigned char key, int x, int y)
{
    /// exit
    if (key == 27)
      std::exit(0);
    /// cancel loading
    if (key == ' ')
        ready = true;
    /// ignore input during loading
    if (!ready)
        return;

    /// disable upper case
    if ((key >= 'A') & (key <= 'Z'))
        key = key - 'A' + 'a';

    /// map keys
    if (key == KEY_LEFT)
        keys[0] = true;
    else if (key == KEY_RIGHT)
        keys[1] = true;
    else if (key == KEY_UP)
        keys[2] = true;
    else if (key == KEY_DOWN)
        keys[3] = true;
}

/**
 * @brief keyboardUp is called when key released
 * @param key is key code
 * @param x is cursor position x
 * @param y is cursor position y
 */
void keyboardUp(unsigned char key, int x, int y)
{
    /// ignore during loading
    if (!ready)
        return;
    /// disable upper case
    if ((key >= 'A') & (key <= 'Z')) {
        key = key - 'A' + 'a';
    }

    /// map keys
    if (key == KEY_LEFT)
        keys[0] = false;
    else if (key == KEY_RIGHT)
        keys[1] = false;
    else if (key == KEY_UP)
        keys[2] = false;
    else if (key == KEY_DOWN)
        keys[3] = false;

    if (key == 'c') {
        clear();
    } else if (key == 'q') {
        poseIndex++;
        if (poseIndex >= poseCount)
            poseIndex = 0;
        loadFrame();
    } else if (key == 'e') {
        poseIndex--;
        if (poseIndex < 0)
            poseIndex = poseCount - 1;
        loadFrame();
    }
}
/**
 * @brief mouseMose is called when the mouse button was clicked
 * @param x is mouse x positon in window
 * @param y is mouse y positon in window
 */
void mouseClick(int button, int status, int x, int y)
{
    if ((button == GLUT_LEFT_BUTTON) && (status == GLUT_DOWN))
        mouseActive = !mouseActive;
    if (mouseActive) {
        mouseX = x;
        mouseY = y;
    }
}

/**
 * @brief mouseMove is called when mouse moved
 * @param x is mouse x positon in window
 * @param y is mouse y positon in window
 */
void mouseMove(int x, int y)
{
    if (mouseActive) {
        mouseX = x;
        mouseY = y;
    }
}

/**
 * @brief reshape rescales window
 * @param w is new window width
 * @param h is new window hegiht
 */
void reshape(int w, int h)
{
   resolution.x = w;
   resolution.y = h;
   if (mouseActive)
   {
       glutWarpPointer(resolution.x / 2, resolution.y / 2);
   }
}

/**
 * @brief main loads data and prepares scene
 * @param argc is amount of arguments
 * @param argv is array of arguments
 * @return exit code
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        LOGI("Usage: ./dataset_viewer <path to the dataset>");
        return 0;
    }

    /// init glut
    glutInit(&argc, argv);
    glutInitWindowSize(960,540);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutCreateWindow("Dataset viewer");
    glutFullScreen();
    shader = new oc::GLSL(kTextureShader[0], kTextureShader[1]);

    /// set handlers
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboardDown);
    glutKeyboardUpFunc(keyboardUp);
    glutMouseFunc(mouseClick);
    glutPassiveMotionFunc(mouseMove);

    /// init camera
    camera = glm::vec3(0, 4, 0);
    pitch = 1.57f;
    yaw = 0;

    /// init dataset
    dataset = new oc::Dataset(argv[1]);
    poseIndex = -1;
    ready = false;
    FILE* file = fopen((dataset->GetPath() + "/state.txt").c_str(), "r");
    if (file) {
        fscanf(file, "%d", &poseCount);
        fclose(file);
    }

    /// start loop
    glutTimerFunc(0, idle, 0);
    glutMainLoop();
    return 0;
}
