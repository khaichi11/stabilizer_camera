# Rencana cetak — Gimbal D435 (sabuk)

Semua file STL di folder ini **sudah dalam posisi cetak**: jangan diputar dan jangan diskala.
Semua part dirancang **tanpa support**; yang ada hanya jembatan pendek (maks. ±14 mm) dan miring 45°.
Ulir (M6 dan 1/4") sudah dimodelkan dengan kelonggaran 0,15 mm.

**Ada 16 file STL, dan 1 file = 1 buah.** Cetak semua file sekali saja, tidak perlu menggandakan apa pun. Part yang dibutuhkan 2 buah sudah dibuat 2 file (`_1` dan `_2`).

## Daftar part

| File | Jumlah | Ukuran (P × L × T, mm) | Bahan | Catatan |
|---|---|---|---|---|
| G_Base.stl | 1 | 127 × 60 × 54 | PETG | Klip sabuk; harus agak lentur, jadi PETG lebih awet dari PLA. Paling penting dicetak kuat (lihat catatan di bawah). |
| G_CamTray.stl | 1 | 114 × 34 × 28 | PETG/PLA | Dudukan kamera |
| G_PitchFrame.stl | 1 | 35 × 23 × 58 | PETG/PLA | Berdiri tinggi; tabung poros di atas |
| G_EspCover.stl | 1 | 27 × 28 × 21 | PETG/PLA | Penutup kotak ESP32 + saluran kabel |
| G_PivotCap.stl | 1 | 14 × 33 × 6 | PETG/PLA | Tutup bantalan poros |
| G_ImuLid.stl | 1 | 24 × 16 × 1,3 | PETG/PLA | Tipis, ada tonjolan kecil di atas |
| G_BeltScrew_1.stl | 1 | 9 × 9 × 14 | PETG/PLA | Baut kenop penjepit sabuk (kiri) |
| G_BeltScrew_2.stl | 1 | 9 × 9 × 14 | PETG/PLA | Baut kenop penjepit sabuk (kanan) |
| G_BeltPad_1.stl | 1 | 13 × 14 × 1,3 | PETG/PLA | Bantalan sabuk, tipis |
| G_BeltPad_2.stl | 1 | 13 × 14 × 1,3 | PETG/PLA | Bantalan sabuk, tipis |
| G_ThumbScrew.stl | 1 | 18 × 18 × 14 | PETG/PLA | Baut 1/4" ke kamera |
| G_ClampScrew.stl | 1 | 14 × 14 × 19 | PETG/PLA | Baut penjepit kamera |
| G_Nut_M6.stl | 1 | 12 × 10 × 5 | PETG/PLA | Mur penjepit kamera |
| G_ClampJaw.stl | 1 | 8 × 11 × 4 | PETG/PLA | Rahang penjepit |
| G_CableScrew.stl | 1 | 12 × 12 × 10 | PETG/PLA | Pengunci colokan listrik |
| G_UsbGate.stl | 1 | 7 × 14 × 4 | PETG/PLA | Baji pengunci kabel kamera |

Volume padat semua part sekitar 110 cm³; filamen sebenarnya lebih sedikit, tergantung pengaturan slicer. Sumber listrik memakai power bank, jadi tidak ada kotak baterai.

## Baut dan mur plastik (semua ikut dicetak)

| Baut/mur | Jumlah | Masuk ke |
|---|---|---|
| G_ThumbScrew (1/4"-20) | 1 | lubang tripod kamera |
| G_ClampScrew (M6) | 1 | G_Nut_M6, mendorong G_ClampJaw |
| G_Nut_M6 (M6) | 1 | dudukan kamera, untuk G_ClampScrew |
| G_BeltScrew_1 dan G_BeltScrew_2 (M6) | 2 | ulir di G_Base (kenop sabuk kiri dan kanan) |
| G_CableScrew (M6) | 1 | ulir di sisi kanan kotak ESP32 (pengunci colokan listrik) |

Totalnya 5 baut + 1 mur plastik. Ulir lawannya sudah ada di G_Base (3 lubang) dan di kamera (lubang tripod). Yang tetap logam hanya sekrup kecil M2 servo.

## Pembagian pelat cetak

Semua part muat dalam **satu pelat 220 × 220 mm** (tata letaknya ada di `D435_gimbal.f3d`, komponen GIMBAL_PRINT_LAYOUT). Kalau bed lebih kecil atau ingin dipisah:

1. **Pelat A — bagian besar:** G_Base + G_CamTray.
2. **Pelat B — bagian gerak dan kecil:** G_PitchFrame, G_EspCover, G_PivotCap, G_ImuLid, G_BeltPad_1, G_BeltPad_2, G_BeltScrew_1, G_BeltScrew_2, G_ThumbScrew, G_ClampScrew, G_Nut_M6, G_ClampJaw, G_CableScrew, G_UsbGate.

## Catatan kekuatan untuk G_Base
- Bagian paling rawan adalah pinggang klip (bagian atas yang menyambung sisi badan dan sisi depan). Sampaikan ke pihak slicing: **G_Base perlu dinding (perimeter) lebih banyak**, karena kekuatan klip datang dari dinding, bukan dari isian.
- Part lain (kotak ESP32, penutup, tutup IMU) ringan dan tidak menahan beban, jadi boleh dicetak biasa.

## Urutan yang disarankan
1. **Cetak uji dulu (kecil, cepat):** G_Nut_M6 + G_BeltScrew_1.
   - Putar baut ke mur: harus masuk dengan tangan, tidak goyang.
   - Kalau terlalu seret atau terlalu longgar, sampaikan ke saya sebelum mencetak pelat A, supaya kelonggaran ulir disesuaikan untuk printer Anda.
2. Pelat B.
3. Pelat A.

## Cek setelah dicetak (sebelum merakit)
- **Bantalan poros (G_Base) + tabung G_PitchFrame:** tabung masuk ke bantalan dari atas, lalu berputar ringan. Kalau seret, amplas tipis.
- **G_PivotCap:** bisa digeser masuk ke rel ekor burung sampai mentok.
- **G_EspCover:** turun lancar di alur depan kotak ESP32.
- **G_ImuLid:** masuk ke rel di bawah dudukan kamera sampai "klik".
- **G_BeltScrew:** masuk ke lubang di G_Base dan bisa diputar sampai bantalan terdorong.
- **G_ThumbScrew:** masuk ke lubang tripod kamera dengan tangan.
