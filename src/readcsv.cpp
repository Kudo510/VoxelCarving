#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

std::vector<double> stringToVector(const std::string& input) {
    std::istringstream iss(input);
    std::vector<double> result;
    double number;

    while (iss >> number) {
        result.push_back(number);
    }

    return result;
}

std::string removeMiddleBracketsAndQuotes(const std::string& input) {
    std::string result = input;

    // Find the index of the middle opening bracket
    size_t openingBracketIndex = result.find('[');
    if (openingBracketIndex == std::string::npos)
        return result; // No opening bracket found

    // Find the index of the middle closing bracket, starting from the opening bracket index
    size_t closingBracketIndex = result.find(']', openingBracketIndex);
    if (closingBracketIndex == std::string::npos)
        return result; // No closing bracket found

    // Find the index of the first double quote after the opening bracket
    size_t quoteIndex = result.find('"', openingBracketIndex);
    if (quoteIndex == std::string::npos)
        return result; // No double quote found

    // Erase the middle opening bracket
    result.erase(openingBracketIndex, 1);

    // Erase the middle closing bracket, considering the offset caused by removing the opening bracket
    result.erase(closingBracketIndex - 1, 1);

    // Erase the first double quote
    result.erase(quoteIndex, 1);

    // Find the index of the next double quote after the previous one
    quoteIndex = result.find('"', quoteIndex);
    if (quoteIndex == std::string::npos)
        return result; // No double quote found

    // Erase the second double quote
    result.erase(quoteIndex, 1);

    return result;
}

std::vector<std::vector<double>> convertCSVtoMatrix(std::ifstream& file) {
    std::string line;

    // Skip the first row if it contains headers
    std::getline(file, line);

    std::vector<std::string> cells;

    // Read the remaining lines and extract the second column cells
    while (std::getline(file, line)) {
        std::istringstream lineStream(line);
        std::string cell;

        // Skip the first column
        std::getline(lineStream, cell, ',');

        // Read the second column cell
        std::getline(lineStream, cell, ',');
        cells.push_back(cell);
    }

    for (std::string& str : cells) {
        str.erase(std::remove_if(str.begin(), str.end(),
            [&](char c) mutable { return c == '[' || c == ']' || c == '"'; }),
            str.end());
    }

    std::vector<std::vector<double>> lines(cells.size());

    for (int i = 0; i < cells.size(); i++) {
        lines[i] = stringToVector(cells[i]);
    }

    return lines;
}
std::vector<cv::Mat> groupVectorsToMatrices(const std::vector<std::vector<double>>& lines) {
    std::vector<cv::Mat> matrices;

    for (int i = 0; i < lines.size(); i += 4) {
        cv::Mat matrix(4, lines[i].size(), CV_64F);

        // Populate the matrix with data from the vectors
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < lines[i + j].size(); k++) {
                matrix.at<double>(j, k) = lines[i + j][k];
            }
        }

        matrices.push_back(matrix);
    }

    return matrices;
}
int main() {
    std::ifstream file("F:\\second semseter\\3D Scanning & Motion Capture\\Final project\\Voxel-Carving\\assets\\around_X.csv");
    std::vector<std::vector<double>> lines = convertCSVtoMatrix(file);
    // Group every 4 vectors into matrices
    std::vector<cv::Mat> matrices = groupVectorsToMatrices(lines);

    // Print the second matrix
    std::cout << "Second Matrix:" << std::endl;
    const cv::Mat& secondMatrix = matrices[15]; // Get the second matrix
    cv::Mat P = secondMatrix(cv::Rect(0, 0, 4, 3)); //only take the first 3 row

    // Print the converted matrix
    std::cout << P << std::endl;  // here we have the same P as we want

    return 0;
}


