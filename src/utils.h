#include "common.h"

cv::Mat getRowFromColumnB(const std::string &filename, int row)
{
    std::ifstream file(filename);
    std::string line, value;

    // Skip the rows until reaching the desired row
    for (int i = 0; i < row; ++i)
    {
        std::getline(file, line);
    }

    // Read the desired row and extract value from column B
    std::getline(file, line);
    std::stringstream ss(line);
    for (int i = 0; i < 1; ++i)
    {
        std::getline(ss, value, ',');
    }

    // Create a cv::Mat object from the value
    cv::Mat rowMat(1, 1, CV_64F);
    rowMat.at<double>(0, 0) = std::stod(value);

    return rowMat;
}
cv::Mat extractRows(const cv::Mat &matrix, int numRows)
{
    cv::Rect roi(0, 0, matrix.cols, numRows);
    cv::Mat extractedRows = matrix(roi).clone();

    return extractedRows;
}
/*
cv::Mat getCellValue(const std::string& filename, int row, int col) {
    std::ifstream file(filename);
    std::string line, value;

    // Skip the rows until reaching the desired row
    for (int i = 0; i < row; ++i) {
        std::getline(file, line);
    }

    // Read the desired row and extract the value from the desired column
    std::getline(file, line);
    std::stringstream ss(line);
    for (int i = 0; i < col; ++i) {
        std::getline(ss, value, ',');
    }

    // Create a cv::Mat object from the value
    cv::Mat cellMat(1, 1, CV_64F);
    cellMat.at<double>(0, 0) = std::stod(value);

    return cellMat;
}*/

cv::Mat getCellValue(const std::string &filename, int row)
{
    std::ifstream file(filename);
    std::string line;

    // Skip the rows until reaching the desired row
    for (int i = 0; i < 20; ++i)
    {
        std::getline(file, line);
    }
    std::cout << "Value in line" << line;

    // Read the desired row and extract the value from the second column (column B)
    std::getline(file, line);
    std::stringstream ss(line);
    std::string value;
    std::getline(ss, value, '\t'); // Skip the first column (column A)
    std::getline(ss, value, '\t'); // Read the value in the second column (column B)

    // Remove any leading/trailing spaces or brackets from the value
    value.erase(std::remove(value.begin(), value.end(), ' '), value.end());
    value.erase(std::remove(value.begin(), value.end(), '['), value.end());
    value.erase(std::remove(value.begin(), value.end(), ']'), value.end());

    // Split the value into individual numbers
    std::stringstream num_ss(value);
    std::vector<double> nums;
    double num;
    while (num_ss >> num)
    {
        nums.push_back(num);
    }

    // Create a cv::Mat object from the numbers
    cv::Mat cellMat(1, nums.size(), CV_64F);
    for (int i = 0; i < nums.size(); ++i)
    {
        cellMat.at<double>(0, i) = nums[i];
    }

    return cellMat;
}
std::vector<double> stringToVector(const std::string &input)
{
    std::istringstream iss(input);
    std::vector<double> result;
    double number;

    while (iss >> number)
    {
        result.push_back(number);
    }

    return result;
}

std::string removeMiddleBracketsAndQuotes(const std::string &input)
{
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

std::vector<std::vector<double>> convertCSVtoMatrix(std::ifstream &file)
{
    std::string line;

    // Skip the first row if it contains headers
    std::getline(file, line);

    std::vector<std::string> cells;

    // Read the remaining lines and extract the second column cells
    while (std::getline(file, line))
    {
        std::istringstream lineStream(line);
        std::string cell;

        // Skip the first column
        std::getline(lineStream, cell, ',');

        // Read the second column cell
        std::getline(lineStream, cell, ',');
        cells.push_back(cell);
    }

    for (std::string &str : cells)
    {
        str.erase(std::remove_if(str.begin(), str.end(),
                                 [&](char c) mutable
                                 { return c == '[' || c == ']' || c == '"'; }),
                  str.end());
    }

    std::vector<std::vector<double>> lines(cells.size());

    for (int i = 0; i < cells.size(); i++)
    {
        lines[i] = stringToVector(cells[i]);
    }

    return lines;
}
std::vector<cv::Mat> groupVectorsToMatrices(const std::vector<std::vector<double>> &lines)
{
    std::vector<cv::Mat> matrices;

    for (int i = 0; i < lines.size(); i += 4)
    {
        cv::Mat matrix(4, lines[i].size(), CV_64F);

        // Populate the matrix with data from the vectors
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < lines[i + j].size(); k++)
            {
                matrix.at<double>(j, k) = lines[i + j][k];
            }
        }

        matrices.push_back(matrix);
    }

    return matrices;
}

void exportModel(char *filename, vtkPolyData *polyData)
{

    /* exports 3d model in ply format */
    vtkSmartPointer<vtkPLYWriter> plyExporter = vtkSmartPointer<vtkPLYWriter>::New();
    plyExporter->SetFileName(filename);
    plyExporter->SetInputData(polyData);
    plyExporter->Update();
    plyExporter->Write();
}

cv::Mat parseLineToMat(const std::string &line)
{
    cv::Mat mat(4, 4, CV_32F);

    std::istringstream iss(line);
    std::string valueStr;
    int index = 0;

    while (std::getline(iss, valueStr, ','))
    {
        valueStr = valueStr.substr(1, valueStr.length() - 2); // Remove the square brackets from each value
        float value = std::stof(valueStr);
        mat.at<float>(index / 4, index % 4) = value;
        index++;
    }

    return mat;
}

std::vector<cv::Mat> readMatricesFromFile(const std::string &file_path)
{
    std::vector<cv::Mat> matrices;

    std::ifstream file(file_path);

    if (file.is_open())
    {
        std::string line;
        while (std::getline(file, line))
        {
            cv::Mat matrix = parseLineToMat(line);
            matrices.push_back(matrix);
        }

        file.close();
    }
    else
    {
        std::cout << "Unable to open the file." << std::endl;
    }

    return matrices;
}

std::string getFileExtension(const std::string &filename)
{
    size_t dotPos = filename.find_last_of(".");
    if (dotPos != std::string::npos && dotPos != filename.length() - 1)
    {
        return filename.substr(dotPos + 1);
    }
    return ""; // Empty string if no extension found
}

bool fileExists(const std::string &filename)
{
    std::ifstream file(filename);
    return file.good();
}

std::string replaceSubstring(const std::string &str, const std::string &oldSubstr, const std::string &newSubstr)
{
    std::string result = str;
    size_t pos = 0;
    while ((pos = result.find(oldSubstr, pos)) != std::string::npos)
    {
        result.replace(pos, oldSubstr.length(), newSubstr);
        pos += newSubstr.length();
    }
    return result;
}

// Function to parse a line and retrieve the value after the delimiter
template <typename T>
T parseValue(const std::string &line, char delimiter = '=')
{
    std::istringstream iss(line);
    std::string key;
    T value;
    if (std::getline(iss, key, delimiter) && iss >> value)
    {
        return value;
    }
    throw std::runtime_error("Error parsing line: " + line);
}