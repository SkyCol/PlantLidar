# include<iostream>
# include<queue>
# include<cmath>
# include<vector>
using namespace std;

class Point
{
public:
    Point(float x = 1, float y = 1);
    friend float distanced(Point& p1, Point& p2);
private:
    float x, y;
};

Point::Point(float x, float y)
{
    this->x = x;    this->y = y;
}

float distanced(Point& p1, Point& p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

struct xy
{
    int x, y;
};


extern "C"
bool c_lmf_bfs(int x,int y,double* rast,int rows,int columns,float res,float radius)
{
    Point start_point(x,y);
    queue<xy>q;
    q.push(xy{x,y}) ;
    vector<vector<int> > vis(rows+1, vector<int>(columns+1, 0)); // Initialize array as 0
    vis[x][y] = 1;
    int dx[] = {-1,1,0,0,-1,-1,1,1};
    int dy[] = {0,0,-1,1,-1,1,-1,1};

    while(!q.empty())
    {
        int cur_x = q.front().x,cur_y=q.front().y;
        q.pop();
        for(int i=0;i<4;i++)
            {
                int nx = cur_x+dx[i];
                int ny = cur_y+dy[i];
                if (0 <= nx && nx<rows && 0<=ny && ny<columns && !vis[nx][ny])
                {
                    q.push(xy{nx,ny});
                    vis[nx][ny] = 1;
                    Point current_point(nx,ny);
                    double dis = distanced(start_point,current_point);
                    if (dis * res > radius)
                    {
                        break;
                    }
                    if(rast[nx*columns+ny]>=rast[x*columns+y])
                    {
                        //while (!q.empty()) q.pop();
                        return 0;
                    }
                }
            }
    }
    return 1;
}