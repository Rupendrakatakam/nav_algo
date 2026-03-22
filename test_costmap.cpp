#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

struct BFSCell { int x, y; float dist; };

int main() {
    int w = 15, h = 15;
    vector<int> grid(w * h, 0);
    for(int i=4; i<=10; i++) grid[i*w + 5] = 1;

    float r_rad = 1.2, i_rad = 3.5;
    vector<float> dist_map(w * h, 1e9f);
    queue<BFSCell> q;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (grid[y * w + x] == 1) {
                dist_map[y * w + x] = 0.0f;
                q.push({x, y, 0.0f});
            }
        }
    }

    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1}, dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    float step[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.41f, 1.41f, 1.41f, 1.41f};

    while (!q.empty()) {
        BFSCell cur = q.front(); q.pop();
        for (int i = 0; i < 8; i++) {
            int nx = cur.x + dx[i], ny = cur.y + dy[i];
            float nd = cur.dist + step[i];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (nd >= i_rad || nd >= dist_map[ny * w + nx]) continue;
            dist_map[ny * w + nx] = nd;
            q.push({nx, ny, nd});
        }
    }
    cout << "dist_map at 7,1 is " << dist_map[1 * w + 7] << endl;
    return 0;
}
