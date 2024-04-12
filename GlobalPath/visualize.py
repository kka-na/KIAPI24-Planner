import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('./PreRound1.csv')

x_left_boundary = []
y_left_boundary = []
x_right_boundary = []
y_right_boundary = []

# 각 포인트에 대해 왼쪽과 오른쪽 경계선의 좌표 계산
for i in range(len(df)):
    # 왼쪽 경계선 좌표 계산
    x_left = df['x'][i] + df['x_normvec'][i] * df['w_left'][i]
    y_left = df['y'][i] + df['y_normvec'][i] * df['w_left'][i]
    x_left_boundary.append(x_left)
    y_left_boundary.append(y_left)
    
    # 오른쪽 경계선 좌표 계산
    x_right = df['x'][i] - df['x_normvec'][i] * df['w_right'][i]
    y_right = df['y'][i] - df['y_normvec'][i] * df['w_right'][i]
    x_right_boundary.append(x_right)
    y_right_boundary.append(y_right)

# 도로 중앙선과 경계선 그리기
plt.figure(figsize=(10, 5))

# 도로 중앙선 그리기 (파란색)
plt.plot(df['x'], df['y'], color='blue', label='Road Centerline')
plt.plot(x_left_boundary, y_left_boundary, color='green', linestyle='-', label='Left Boundary')
plt.plot(x_right_boundary, y_right_boundary, color='red', linestyle='-', label='Right Boundary')
# 그래프 설정
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Road Visualization with Boundaries')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()