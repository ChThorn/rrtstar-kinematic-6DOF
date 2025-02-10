#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>

int main() {
    // Create a data file for 3D plotting
    std::ofstream dataFile("data3d.dat");
    for (double x = -10; x <= 10; x += 0.5) {
        for (double y = -10; y <= 10; y += 0.5) {
            double z = sin(sqrt(x * x + y * y));
            dataFile << x << " " << y << " " << z << "\n";
        }
        dataFile << "\n"; // Separate rows for Gnuplot
    }
    dataFile.close();

    // Use Gnuplot for interactive plotting
    std::ofstream gnuplotScript("plot3d.gp");
    gnuplotScript << "set terminal qt\n";  // Use interactive Qt terminal
    gnuplotScript << "set dgrid3d\n";
    gnuplotScript << "splot 'data3d.dat' with lines\n";
    gnuplotScript.close();

    system("gnuplot -p plot3d.gp");  // The -p flag keeps the plot open until you close it manually

    return 0;
}
