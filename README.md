# lmx-clang-samples

C++ samples working on LMX (Linux version of WMX). 

This project demonstrates the use of the LMX library for managing memory logs, updating log data, and visualizing feedback data using `gnuplot`.

---

## Requirements

- **Operating System**: 
  - Tested on **Ubuntu 24.04 LMX version**.
  - Should also work on other Linux distributions with minor adjustments.

- **Dependencies**:
  - **C++17 or later**: Required for modern C++ features like `std::thread`, `std::unique_ptr`, and `std::atomic`.
  - **LMX Library**: The Linux version of WMX.
  - **gnuplot**: For data visualization.
  - **nlohmann/json**: For JSON handling.
  - **xtl/xbase64**: For base64 encoding.
  - **xcpp/xdisplay**: For displaying plots in Jupyter-like environments.

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/kyoungje/lmx-clang-samples.git
   cd lmx-clang-samples