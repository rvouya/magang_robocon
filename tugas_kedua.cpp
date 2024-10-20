#include <iostream>
#include <cmath>
#include <fstream>
#include <string>

using namespace std;

#define PI 3.14159265359
#define rad PI / 180.0

class Omniwheel {
private:
    float matrix[4][3];
    int speed_motor[4];

public:
    Omniwheel() {
        matrix[0][0] = cosf(45.0 * rad); 
        matrix[0][1] = sinf(45.0 * rad); 
        matrix[0][2] = 1;
        matrix[1][0] = cosf(135.0 * rad);
        matrix[1][1] = sinf(135.0 * rad); 
        matrix[1][2] = 1;
        matrix[2][0] = cosf(225.0 * rad); 
        matrix[2][1] = sinf(225.0 * rad); 
        matrix[2][2] = 1;
        matrix[3][0] = cosf(315.0 * rad); 
        matrix[3][1] = sinf(315.0 * rad); 
        matrix[3][2] = 1;
    }

    void SpeedCalc(int speed_vector[3]) {
        for (int i = 0; i < 4; i++) {
            speed_motor[i] = 0;
        }
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                speed_motor[i] += matrix[i][j] * speed_vector[j];
            }
        }
    }

    void simpanKeFile(const string &filename) {
        ofstream file(filename);
        for (int i = 0; i < 4; i++) {
            file << "Kecepatan roda " << i + 1 << " : " << speed_motor[i] << endl;
        }
        file.close();
    }

    void tampilkanKecepatanMotor() {
        for (int i = 0; i < 4; i++) {
            cout << "Speed motor " << i << " : " << speed_motor[i] << endl;
        }
    }
};

int main() {
    Omniwheel roda;

    int input[3];
    cout << "Speed X: ";
    cin >> input[0];
    cout << "Speed Y: ";
    cin >> input[1];
    cout << "Speed Z: ";
    cin >> input[2];

    roda.SpeedCalc(input);
    roda.tampilkanKecepatanMotor();
    roda.simpanKeFile("catat.txt");

    return 0;
}