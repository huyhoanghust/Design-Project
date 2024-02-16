#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define NUM_PARTICLES 100
#define SIGMA 1.0

// Cấu trúc lưu trữ trạng thái particle
typedef struct {
    double distance;
} Particle;

// Hàm cập nhật trọng số của particle dựa trên khoảng cách quan sát và khoảng cách dự báo
double update_weight(double observed_distance, double predicted_distance) {
    // Tính trọng số dựa trên phân phối Gaussian
    double weight = exp(-pow(observed_distance - predicted_distance, 2) / (2 * pow(SIGMA, 2)));
    return weight;
}

// Hàm lấy mẫu lại dựa trên trọng số của các particle
void resample(Particle* particles, double* weights) {
    // Tạo mảng tạm để lưu trữ particle mới sau khi lấy mẫu
    Particle resampled_particles[NUM_PARTICLES];
    
    // Tạo một mảng để lưu trữ tổng tích lũy của trọng số
    double cumulative_weights[NUM_PARTICLES];
    cumulative_weights[0] = weights[0];
    
    // Tính tổng tích lũy của trọng số
    int i;
    for (i = 1; i < NUM_PARTICLES; i++) {
        cumulative_weights[i] = cumulative_weights[i - 1] + weights[i];
    }
    
    // Lấy mẫu mới dựa trên trọng số
        i;
    for (i = 0; i < NUM_PARTICLES; i++) {
        double rand_num = (double)rand() / RAND_MAX; // Số ngẫu nhiên trong khoảng [0, 1]
        int j = 0;
        while (rand_num > cumulative_weights[j] && j < NUM_PARTICLES - 1) {
            j++;
        }
        resampled_particles[i] = particles[j];
    }
    
    // Sao chép particle mới vào mảng particle ban đầu
        i;
    for (i = 0; i < NUM_PARTICLES; i++) {
        particles[i] = resampled_particles[i];
    }
}

int main() {
    // Đọc dữ liệu từ file .csv
    char file_path[] = "E:/DATK/REPORT/data_kalman/data.csv";
    FILE* file = fopen(file_path, "r");
    if (file == NULL) {
        printf("dont open file %s\n", file_path);
        return 1;
    }

 // Khởi tạo particle ban đầu
    Particle particles[NUM_PARTICLES];
    double weights[NUM_PARTICLES];
    int i;
    for (i = 0; i < NUM_PARTICLES; i++) {
        particles[i].distance = (double)rand() / RAND_MAX; // Khởi tạo khoảng cách của particle ngẫu nhiên
        weights[i] = 1.0 / NUM_PARTICLES; // Trọng số ban đầu cho các particle
    }

    // Đọc dữ liệu từ file .csv và thực hiện bộ lọc particle
    double observed_distance;
    int t;
    for (t = 0; fscanf(file, "%lf", &observed_distance) != EOF; t++) {
        // Cập nhật trọng số của particle dựa trên khoảng cách quan sát và khoảng cách dự báo
        for (i = 0; i < NUM_PARTICLES; i++) {
            weights[i] = update_weight(observed_distance, particles[i].distance);
        }

        // Chuẩn hóa trọng số và lấy mẫu lại các particle dựa trên trọng số
        double sum_weights = 0;
        for (i = 0; i < NUM_PARTICLES; i++) {
            sum_weights += weights[i];
        }
        for (i = 0; i < NUM_PARTICLES; i++) {
            weights[i] /= sum_weights;
        }
        resample(particles, weights);

        // Ước lượng khoảng cách từ các particle có trọng số cao nhất (ví dụ: trung bình)
        double estimated_distance = 0;
        for (i = 0; i < NUM_PARTICLES; i++) {
            estimated_distance += weights[i] * particles[i].distance;
        }

        // In kết quả ước lượng khoảng cách
        printf("Thoi diem %d: Uoc luong khoang cach = %.2lf\n", t, estimated_distance);
    }

    fclose(file);
    return 0;
}
