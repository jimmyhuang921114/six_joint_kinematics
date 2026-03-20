

## 專案檔案

- `kinematics.py`：完整版範例
- `kinematics_sample.py`：教學骨架版
- `requirements.txt`：Python 套件需求
- `.env.example`：環境變數範例

## 安裝方式

先建立虛擬環境：

```bash
python3 -m venv .venv
source .venv/bin/activate
```

安裝套件：

```bash
pip install -r requirements.txt
```
### 執行完整版
```bash
python kinematics.py
```

### 執行 sample 教學版
```bash
python kinematics_sample.py
```

## 程式依賴說明

這個專案實際用到的第三方套件只有：

- `numpy`
- `matplotlib`

`math` 是 Python 內建模組，不需要額外安裝。

## 注意事項


```text
project/
├── kinematics.py
├── kinematics_2024.py
├── kinematics_sample.py
├── requirements.txt
├── .env.example
└── README.md
```

## 快速開始

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
python kinematics.py
```
