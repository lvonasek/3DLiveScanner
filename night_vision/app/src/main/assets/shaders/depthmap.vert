uniform float u_PointSize;
uniform float u_Rotation;
uniform float u_Scheme;

attribute vec4 a_Position;

varying vec4 v_Color;

void main() {
   float c = a_Position.w;
   float d = 1.0 - a_Position.z * 0.25;
   float e = a_Position.z;
   if ((u_Scheme > -0.5) && (u_Scheme < 0.5)) v_Color = vec4(d, c, 0.0, 1.0);
   if ((u_Scheme > 0.5) && (u_Scheme < 1.5)) v_Color = vec4(0.0, c, d, 1.0);
   if ((u_Scheme > 1.5) && (u_Scheme < 2.5)) v_Color = vec4(c, d, c, 1.0);
   if ((u_Scheme > 2.5) && (u_Scheme < 3.5)) v_Color = vec4(d, d, d, 1.0);
   if ((u_Scheme > 3.5) && (u_Scheme < 4.5)) v_Color = vec4(e, c, 1.0 - e - c, 1.0);
   if (abs(u_Rotation - 180.0) < 1.0) gl_Position = vec4(-a_Position.y, a_Position.x, a_Position.z * 0.01, 1.0);
   if (abs(u_Rotation - 90.0) < 1.0) gl_Position = vec4(-a_Position.x, -a_Position.y, a_Position.z * 0.01, 1.0);
   if (abs(u_Rotation - 0.0) < 1.0) gl_Position = vec4(a_Position.y, -a_Position.x, a_Position.z * 0.01, 1.0);
   if (abs(u_Rotation + 90.0) < 1.0) gl_Position = vec4(a_Position.x, a_Position.y, a_Position.z * 0.01, 1.0);
   gl_PointSize = u_PointSize;
}
