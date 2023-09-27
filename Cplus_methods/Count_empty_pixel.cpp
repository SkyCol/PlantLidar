# include<iostream>
using namespace std;

int dx[] = { -1,1,0,0 };
int dy[] = { 0,0,-1,1 };

int dfs_zeros(double* rast, int x, int y, int columns, int rows, int counts)
{
    counts++;
    rast[x * columns + y] = 1;
    int newx, newy;
    for (int direct = 0; direct < 4; direct++)
    {
        newx = x + dx[direct];
        newy = y + dy[direct];
        if (rast[x * columns + y] != 0 || newx<1 || newx>rows || newy<1 || newy>columns)
            continue;
        counts = dfs_zeros(rast, newx, newy, columns, rows, counts);
    }
    return counts;
}

extern "C"
int count_surrounding_zeros(double* rast, int rows, int columns)
{
    int counts = 0;
    for (int i = 1; i < rows - 1; i++)
    {
        for (int j = 1; j < columns - 1; j++)
        {
            if (rast[i * columns + j] == 0)
            {
                counts = dfs_zeros(rast, i, j, columns, rows, counts);
            }
        }
    }
    return counts;
}