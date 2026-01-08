# 🔐 Advanced Encryption Standard (AES) Implementation in C

## 📌 Overview
This project implements the **Advanced Encryption Standard (AES)** algorithm in **C**, focusing on secure and efficient data encryption and decryption. AES is a widely used symmetric-key cryptographic standard adopted by governments and industries worldwide for protecting sensitive data.

The implementation demonstrates a strong understanding of **cryptographic algorithms, low-level programming, and secure data handling**, making it suitable for **embedded systems, firmware, and security-focused applications**.

---

## ⚙️ Features
- AES encryption and decryption
- Fixed-size block processing (128-bit)
- Key expansion (Key Scheduling)
- Support for multiple AES rounds
- Lightweight implementation suitable for embedded systems

---

## 🧠 AES Algorithm Overview
AES operates on a **128-bit data block** and uses a **symmetric secret key** for both encryption and decryption.

### Core AES Operations
- **SubBytes:** Non-linear byte substitution using S-box
- **ShiftRows:** Row-wise permutation for diffusion
- **MixColumns:** Column mixing using Galois Field arithmetic
- **AddRoundKey:** XOR operation with round keys

The encryption process consists of **multiple rounds**, each strengthening security against cryptanalysis.

---

## 🧩 Key Expansion (Key Scheduling)
- Original cipher key expanded into multiple round keys
- Ensures unique transformation at each encryption round
- Implemented using:
  - Byte substitution
  - Rotation
  - XOR with round constants

---

## 💻 Software Implementation
**Language:** C  
**Paradigm:** Procedural  
**Target Use Cases:** Embedded systems, firmware, secure communication

### Key Highlights
- Bitwise operations for performance optimization
- Modular function design
- Efficient memory usage
- Suitable for resource-constrained systems

📂 **Source code available in `https://github.com/AhsanShadabPatel/Handheld-Gaming-Console/tree/main/code`**

---

## 🛡️ Security Considerations
- Uses standard AES transformations
- Avoids dynamic memory allocation
- Deterministic execution suitable for embedded targets
- Can be extended with:
  - Padding schemes
  - Modes of operation (CBC, ECB, CTR)

---

## 📄 Documentation
📘 **Detailed Report:**  
https://github.com/AhsanShadabPatel/Handheld-Gaming-Console/tree/main/docs

---

## 🧠 Key Skills Demonstrated
- Cryptography fundamentals
- AES algorithm implementation
- C programming & bitwise operations
- Secure data handling
- Embedded-friendly software design
- Algorithm optimization

---


