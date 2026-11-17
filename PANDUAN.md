# Panduan gimbal sabuk D435

**Bahasa Indonesia** | [English](GUIDE.en.md)

Berkas STL tersedia di folder `stl`, model lengkap di `fusion/D435_gimbal.f3d`, firmware di `firmware`, dan aplikasi pengatur di `docs`. Aplikasi pengatur juga tertanam di dalam firmware dan dibuka melalui browser.

Istilah yang dipakai dalam panduan ini:
- **Sudut atas-bawah** (*pitch*): seberapa jauh kamera mengarah ke bawah dari garis datar. Bawaannya 35°.
- **Sudut kanan-kiri** (*roll*): kemiringan kamera ke kanan atau ke kiri. Targetnya 0° agar cakrawala datar.
- **Servo putar atas-bawah** mengarahkan kamera ke atas atau ke bawah dan berada di blok kiri G_Base.
- **Servo putar kanan-kiri** memiringkan kamera ke kanan atau ke kiri (meluruskan cakrawala, bukan menoleh) dan berada di rangka, di belakang kamera.

## Prinsip desain
- **Semua kabel cukup diletakkan, lalu ditutup.** Tidak ada kabel yang harus dimasukkan ke lubang tertutup.
  - Kabel servo putar atas-bawah masuk ke alur di atas pelat.
  - Kabel servo putar kanan-kiri dan kabel IMU diletakkan ke celah tabung poros kanan, lalu ditutup G_PivotCap.
  - Semua kabel masuk ke kotak ESP32 melalui celah samping, lalu ditutup G_EspCover.
- **Elektronik dapat disolder lebih dahulu**, misalnya untuk diuji di meja sebelum hasil cetak tersedia. Setelah hasil cetak tersedia, servo, IMU, dan ESP32 yang sudah tersambung tinggal dipindahkan dan kabelnya diletakkan ke alurnya. Tidak ada sambungan yang perlu dilepas.
- **Semua penutup digeser searah kabel** atau terbuka di sisi kabel, sehingga tidak tersangkut kabel. Hal ini sudah diperiksa di Fusion.

## Daftar belanja
- 2× servo MG90S **180°** (gigi logam). Horn dan sekrup kecilnya sudah termasuk dalam paket. Jangan membeli versi 360° (*continuous*), karena versi tersebut tidak dapat berhenti pada sudut tertentu.
- 1× MPU6050 (modul GY-521, ±21 × 16 mm, dua lubang M3).
- 1× ESP32-C3 Super Mini.
- 1× kapasitor elektrolit **100–470 µF** dengan tegangan berapa pun ≥ 6,3 V dan badan maksimal **Ø6,3 × 11 mm** agar muat di kotak ESP32. Contoh yang sesuai: 100 µF 25 V. Kapasitor 470 µF 50 V atau 1000 µF umumnya terlalu besar.
- **Modul USB-C tambahan dan resistor tidak diperlukan.** Kabel daya dicolokkan langsung ke port USB-C ESP32-C3.
- **Power bank** 5 V ≥ 2 A dan kabel **USB-A ke USB-C**. Pilih power bank yang memiliki mode arus kecil agar tidak mati sendiri.
- Kepala colokan USB-C kabel daya berukuran maksimal 14 × 8 mm (kabel biasa ±12 × 7 mm sudah sesuai).
- Kabel USB-C yang tipis dan lentur (Ø4–5 mm) untuk D435.
- Sekrup *self-tapping* M2: **2 buah M2 × 6** (horn servo putar atas-bawah) dan **2 buah M2 × 4** (horn servo putar kanan-kiri, jangan lebih panjang).
- Kabel untuk IMU: kabel jumper **female-male 20 cm** (kepalanya dipotong) atau kabel silikon serabut 28 AWG empat warna. Jangan memakai kabel tunggal (*solid*), karena kabel IMU terus tertekuk pada sambungan yang bergerak.
- Selongsong bakar (*heat shrink*) dan sedikit vaselin atau gemuk silikon untuk poros.
- Opsional: 2 kabel ties dengan lebar ≤ 4,8 mm.

## Bagian yang dicetak (semua berkas sudah dalam posisi cetak)

Pembagian pelat dan urutan pencetakan tercantum di `stl/RENCANA_CETAK.md`.

| Berkas | Jumlah | Keterangan |
|---|---|---|
| G_Base | 1 | Klip sabuk, dudukan servo putar atas-bawah, bantalan poros kanan, dan kotak ESP32 |
| G_PitchFrame | 1 | Rangka servo putar kanan-kiri, dengan tabung poros berlubang di sisi kanan |
| G_PivotCap | 1 | Penutup bantalan poros kanan (digeser masuk dan menutup kabel di tabung) |
| G_CamTray | 1 | Dudukan kamera, penjepit, dan kantong IMU di bagian bawah |
| G_ImuLid | 1 | Penutup geser kantong IMU |
| G_ThumbScrew | 1 | Baut 1/4" ke lubang tripod kamera |
| G_ClampScrew | 1 | Baut pengencang penjepit kamera |
| G_ClampJaw | 1 | Rahang penjepit kamera |
| G_Nut_M6 | 1 | Mur penjepit kamera |
| G_EspCover | 1 | Penutup kotak ESP32, saluran kabel dari poros, dan pengunci kabel kamera pada bagian mukanya |
| G_UsbGate | 1 | Baji geser pengunci kabel USB kamera |
| G_CableScrew | 1 | Baut pengunci kepala colokan daya (dari sisi kanan kotak) |
| G_BeltScrew_1, G_BeltScrew_2 | **2** (2 berkas) | Baut kenop penjepit sabuk (diputar dari depan) |
| G_BeltPad_1, G_BeltPad_2 | **2** (2 berkas) | Bantalan penekan sabuk |

Folder `stl` berisi 16 berkas STL, dan setiap berkas mewakili satu bagian, sehingga cukup mencetak semua berkas satu kali. Semua baut dan mur besar ikut dicetak: G_ThumbScrew, G_ClampScrew dan G_Nut_M6, G_BeltScrew ×2, serta G_CableScrew. Yang tetap berbahan logam hanya sekrup kecil M2 untuk servo.

Klip sabuk memiliki lidah sisi badan setebal 3 mm, dinding depan 4,5 mm, dan pinggang atas (penyambung keduanya) 6,5 mm dengan sudut dalam yang diisi 1,2 mm agar tidak mudah retak. Alur kabel di atasnya hanya sedalam 2 mm, sehingga pinggangnya tetap tebal. Bagian yang hanya berfungsi sebagai pengisi dibuat berongga untuk menghemat filamen, yaitu penyangga dan bagian belakang blok servo (terbuka dari bawah) serta lantai depan kotak ESP32. Semua rongga tetap dapat dicetak tanpa penyangga.

Saran: gunakan PETG apabila tersedia, karena klip sabuk lebih awet ketika melentur. Cetak dengan 3–4 perimeter dan *infill* 25–40%. Ulir sudah diberi kelonggaran 0,15 mm. Jika ulir terlalu seret, cetak ulang mur dan bautnya saja terlebih dahulu.

## Urutan perakitan
1. **Posisikan servo di tengah terlebih dahulu.**
   - Unggah firmware dan sambungkan servo (sementara atau langsung dengan kabel final). Selama belum dikalibrasi, firmware menahan kedua servo di posisi tengah (1500 µs). Setelah kalibrasi, mode **Tengah** di aplikasi melakukan hal yang sama.
   - Jangan memutar as servo setelah itu.
2. **Servo putar atas-bawah (MG90S kiri).**
   - Masukkan ke kantong blok kiri G_Base dari sisi kanan, lalu pasang dua sekrup kupingnya.
   - Letakkan kabelnya ke celah atas blok, lalu ke alur di atas pelat (ada bibir kecil yang menahan), kemudian ke alur tegak di depan pelat hingga ke kotak ESP32.
   - Blok memiliki celah kabel di **kedua** ujung servo, sehingga sesuai di mana pun kabel servo Anda keluar.
3. **Servo putar kanan-kiri ke rangka (G_PitchFrame).**
   - Masukkan servo dari depan hingga kupingnya menempel, lalu pasang dua sekrup kupingnya.
   - Arahkan kabelnya ke ruang kecil di balik pelat kanan rangka.
   - Jika kabel servo Anda keluar di ujung dekat as (kiri), lewatkan kabel ke belakang rangka lalu ke bawahnya, kemudian masukkan melalui celah bawah yang sama dengan kabel IMU.
   - Pasang horn servo putar atas-bawah ke kantong di pelat kiri rangka dengan dua sekrup **M2 × 6**. Lengan horn menghadap ke atas.
4. **Dudukan kamera (G_CamTray) ke servo putar kanan-kiri.**
   - Pasang horn ke kantong di belakang dudukan dengan dua sekrup **M2 × 4**. Jangan memakai M2 × 6 di sini, karena ujungnya dapat menyentuh punggung kamera.
   - Tekan dudukan ke as servo, lalu kencangkan sekrup tengah melalui lubang di pelat belakang dudukan.
5. **IMU.**
   - **Solder terlebih dahulu** empat kabel ke baris pin GY-521 **dari bawah papan** (sisi tanpa chip), tanpa pin header. Jika header sudah tersolder, potong pinnya hingga rata.
   - Masukkan GY-521 ke kantong di bawah dudukan dengan chip menghadap ke atas (ke arah kamera), dua lubang M3 masuk ke dua pasak, dan baris pin berada di belakang.
   - Penutup IMU sengaja tidak menutupi baris pin, sehingga solderan dan kabelnya bebas. Kabel langsung masuk ke alur di bawah dudukan menuju bagian tengah (dekat as servo putar kanan-kiri).
   - Geser G_ImuLid dari depan ke rel di bawah kantong hingga berbunyi "klik". Sensor terkunci oleh pasak dan penutup tanpa perekat dua sisi.
   - Jika lubang modul Anda tidak sesuai dengan pasak (berbeda merek), potong pasaknya. Penutup tetap menjepit papan.
6. **Pasang rangka ke G_Base.**
   - Pegang rangka (sudah lengkap dengan servo putar kanan-kiri dan dudukan kamera) dalam posisi kamera mengarah ke bawah, digeser ±3,5 mm ke kanan.
   - Turunkan dari atas hingga tabung kanannya masuk ke bantalan terbuka di lengan kanan G_Base.
   - Geser ke kiri hingga horn masuk ke as servo putar atas-bawah.
   - Kencangkan sekrup tengah horn. Obeng masuk melalui lubang tabung dari kanan, sebelum kabel dimasukkan.
7. **Kabel melalui poros kanan (cukup diletakkan).**
   - Kabel IMU keluar dari alur di bawah dudukan (dekat as servo putar kanan-kiri), lalu masuk ke rangka melalui **celah bawah** di samping pelat kanan, sehingga kabel lewat di bawah dan tidak terlihat.
   - Di dalam rangka, kabel IMU bergabung dengan kabel servo putar kanan-kiri. Keduanya diletakkan ke **celah tabung** yang menghadap ke atas.
   - Keluarkan ujungnya di sisi kanan tabung.
   - Geser **G_PivotCap** dari kanan ke rel ekor burung di atas bantalan hingga mentok. Tabung dan kabel kini tertutup.
8. **Kamera.**
   - Masukkan G_Nut_M6 dari atas ke tiang penjepit di ujung kiri dudukan.
   - Letakkan G_ClampJaw di antara dua rel kecil, lalu putar G_ClampScrew dari luar hingga ujung bautnya masuk ke lubang di belakang rahang.
   - Letakkan kamera dengan port USB-C di sebelah kanan, lalu kencangkan G_ThumbScrew dari bawah ke lubang tripod (celahnya ±2 mm, sehingga pasti bertemu ulir kamera).
   - Putar G_ClampScrew hingga rahang mendorong kamera ke penahan rendah di sebelah kanan.
9. **ESP32-C3.**
   - Masukkan ESP32-C3 dari depan kotak, sejajar pelat, dengan **port USB-C di bawah** dan sisi komponen menghadap ke depan. Dorong hingga menempel di belakang; port USB-C tepat berada di atas lubang di lantai kotak.
   - Kabel servo putar atas-bawah masuk dari belakang di atas papan. Kabel dari poros masuk melalui **celah di sisi kiri kotak**; celah ini terbuka ke depan, sehingga kabel cukup diletakkan.
10. **Penutup G_EspCover.**
    - Turunkan dari atas ke alur di depan kotak. Saluran kabelnya terbuka di bawah, sehingga penutup turun melewati kabel dari poros tanpa tersangkut.
    - Setelah turun penuh:
      - kotak tertutup rata;
      - kabel dari poros hingga kotak tertutup saluran;
      - G_PivotCap tertahan dan tidak dapat bergeser keluar.
11. **Kabel daya.**
    - Colokkan kabel USB-C dari **bawah** kotak hingga masuk ke port ESP32. Kepala colokan masuk ±9 mm ke lubang dan dikunci dengan baut.
    - Putar G_CableScrew dari **sisi kanan** kotak hingga menekan kepala colokan. Cukup dikencangkan dengan tangan.
    - Kabel ini juga dipakai untuk mengunggah firmware, sehingga penutup tidak perlu dilepas.
12. **Kabel USB kamera.**
    - Jepit pada klip bulat di bawah ujung kanan dudukan kamera, lalu sisakan lengkungan longgar ±4 cm ke atas.
    - Tekan kabel ke alur tegak di **muka penutup ESP32** (masuk dari atas, keluar di bawah ke arah laptop), lalu turunkan G_UsbGate. Makin ke bawah, makin kuat baji menjepit (sesuai untuk kabel Ø4–5 mm).
    - Tarikan kabel dari arah laptop tertahan di sini, sehingga tidak menarik kamera dan tidak melawan servo.
13. **Pemeriksaan.** Gerakkan gimbal dengan tangan hingga batas-batasnya (sudut atas-bawah 10–60°, sudut kanan-kiri ±25°). Gerakan harus ringan dan tidak ada kabel yang tertarik.

## Pemasangan ke sabuk
- **Baut penjepit (cara utama).**
  1. Kendurkan **kedua** kenop G_BeltScrew: satu di kiri (di antara blok servo putar atas-bawah dan rangka), satu di kanan (di atas penutup poros).
  2. Masukkan klip dari atas sabuk, lalu tekan turun hingga pengait bawah berbunyi "klik" di bawah tepi sabuk.
  3. Putar kedua kenop **dari depan** hingga bantalan G_BeltPad menekan sabuk ke sisi badan. Cukup dikencangkan dengan tangan.

  Bantalan duduk di ceruk dinding depan, sehingga sabuk setebal 1,5–5 mm dapat dijepit tanpa ganjal dan tanpa bagian yang menonjol ke belakang.
- **Kabel ties (cadangan, di kedua ujung).** Apabila klip retak, kabel ties di kedua ujung tetap mengikat dudukan ke sabuk.
  - Kiri: lubang atas dan bawah di ujung kiri pelat.
  - Kanan: lubang atas di atas kotak ESP32 dan lubang bawah di bawahnya. Di antara keduanya, kabel ties melewati saluran tegak di dalam kotak, menempel pada pelat (cukup didorong dari bawah ke atas).
  1. Masukkan kabel ties dari depan melalui lubang atas.
  2. Turunkan di belakang klip (sisi badan).
  3. Masukkan kembali ke depan melalui lubang bawah.
  4. Kunci di depan bawah, lalu tarik hingga kencang.

  Tarikan kabel ties menekan bagian tebal di antara dua lubang, bukan tepi pelat.
- Pasang di tengah depan (sekitar pusar), bukan di samping pinggul. Di samping pinggul, gerakan paha saat melangkah paling besar.

## Sekrup servo
Di sisi servo, as servo itu sendiri menjadi poros gimbal. Pengikatnya:
- **Badan servo:** dua sekrup M2 melalui kuping servo ke lubang pilot (Ø1,6) di dudukan atau rangka.
- **Horn:** horn masuk ke kantongnya, lalu:
  - dua sekrup melalui lubang horn ke lubang pilot (Ø1,5): **M2 × 6 untuk servo putar atas-bawah, M2 × 4 untuk servo putar kanan-kiri**;
  - sekrup tengah bawaan servo dipasang ke as servo. Sekrup inilah yang mengunci horn agar tidak lepas dari as.

Totalnya sepuluh sekrup logam: empat di kuping servo, dua sekrup tengah, dan empat di horn. Horn juga terbenam 2 mm di kantongnya, sehingga gaya putar servo ditahan oleh dinding kantong, bukan hanya oleh sekrup kecil.

## Sambungan kabel (disolder, tanpa konektor)
Semua kabel disolder langsung tanpa konektor, karena konektor mudah lepas apabila terus berguncang, dan semua jalur kabel di sini cukup diletakkan. Kabel jumper boleh dipakai asalkan kepalanya dipotong.

![Skema rangkaian](gambar/skema_rangkaian.png)

Skema yang sama tersedia di aplikasi, menu **Perangkat**.

**Letak komponen:**
- **ESP32-C3:** berdiri sejajar pelat di kotak tipis sisi kanan G_Base dengan port USB-C menghadap ke bawah. Letaknya tetap dan berada tepat di samping poros kanan, sehingga kabelnya pendek.
- **IMU:** di kantong bawah dudukan kamera. IMU harus ikut bergerak bersama kamera, karena yang dijaga tetap lurus adalah kamera.
- **Servo putar atas-bawah:** di blok kiri dudukan. **Servo putar kanan-kiri:** di rangka, di belakang kamera.

**Jalur kabel:**
- **Servo putar atas-bawah:** celah atas blok kiri → alur di atas pelat → alur tegak di depan pelat → masuk ke kotak ESP32 dari belakang.
- **IMU:** alur di bawah dudukan kamera → keluar dekat as servo putar kanan-kiri → celah bawah rangka → bergabung dengan kabel servo putar kanan-kiri.
- **Servo putar kanan-kiri dan IMU:** celah tabung poros kanan (ditutup G_PivotCap) → saluran di G_EspCover → celah samping kotak ESP32. Karena melewati sumbu putar, kabel hampir tidak tertekuk saat gimbal bergerak.
- **USB kamera:** klip di bawah dudukan → lengkungan longgar → pengunci geser di muka penutup ESP32 → turun ke laptop.
- **Daya:** dari bawah kotak, langsung ke port USB-C ESP32.

**Perkiraan panjang kabel:** servo putar atas-bawah ±15 cm (dipotong dari kabel bawaan), servo putar kanan-kiri ±8 cm, IMU ±18 cm.

**Langkah penyolderan:**
1. **Jangan memasang pin header pada ESP32-C3.** Solder kabel dari sisi komponen langsung ke lubang pinnya; di balik papan hanya tersedia celah 1 mm.
2. **Kapasitor (100–470 µF):** solder kakinya langsung ke pin 5V (kaki panjang, +) dan G (kaki pendek, −). Badannya berdiri di depan papan, di sisi kanan.
3. **Servo:** potong colokannya, kupas, lalu solder:
   - merah → 5V
   - cokelat → G
   - oranye → GPIO3 (servo putar atas-bawah) / GPIO4 (servo putar kanan-kiri)
4. **IMU (GY-521):**
   - VCC → 3V3
   - GND → G
   - SDA → GPIO6
   - SCL → GPIO7

   Solder dari bawah papan tanpa header. Di alur bawah dudukan kamera (dalam 1,9 mm), keempat kabel disusun **berjajar**, bukan ditumpuk. Setelah keluar dari dudukan, kabel boleh dipilin.

   Satu pin 5V dan satu pin G cukup untuk semuanya. Kabel merah kedua servo disatukan terlebih dahulu (sambungan Y yang dibungkus selongsong bakar), lalu satu ujungnya disolder ke 5V. Kabel cokelat kedua servo dan GND IMU disatukan dengan cara yang sama ke G.

   GPIO3 dan GPIO4 yang tertukar, atau IMU yang terpasang terbalik, tidak menjadi masalah. Atur saja di aplikasi, menu Kalibrasi → Servo (Kabel servo, Sumbu MPU6050).
5. Bungkus setiap sambungan dengan selongsong bakar.

| Pin ESP32-C3 | Tersambung ke |
|---|---|
| Port USB-C (bawah kotak) | kabel dari power bank (dikunci G_CableScrew dari kanan), juga untuk mengunggah firmware |
| 5V | kabel merah kedua servo dan kaki positif kapasitor |
| G | kabel cokelat kedua servo, kaki negatif kapasitor, dan GND IMU |
| 3V3 | VCC IMU |
| GPIO3 | kabel oranye servo putar atas-bawah |
| GPIO4 | kabel oranye servo putar kanan-kiri |
| GPIO6 / GPIO7 | SDA / SCL IMU |

**Tentang daya:**
- Pin 5V ESP32-C3 Super Mini mendapat 5 V dari port USB-C-nya, sehingga servo ikut mendapat daya dari kabel yang sama. Arus servo tidak melewati chip ESP32 maupun regulator 3,3 V-nya.
- Pada pemakaian normal, kedua servo hanya melakukan koreksi kecil: totalnya sekitar 0,1–0,5 A, dengan lonjakan sesaat sekitar 1 A. Power bank 2 A sudah cukup.
- Pada sebagian papan, jalur USB ke pin 5V melewati dioda kecil. Ukur tegangan antara pin 5V dan G dengan multimeter saat power bank tersambung: sekitar 5,0 V berarti tersambung langsung, sedangkan 4,6–4,8 V berarti melewati dioda. Keduanya aman untuk gimbal ini. Firmware membatasi gerak servo agar servo tidak pernah tertahan menekan batas.
- Gunakan kabel **USB-A ke USB-C**. Sebagian papan Super Mini tiruan tidak memiliki resistor CC, sehingga kabel USB-C ke USB-C dari pengisi daya atau power bank kadang tidak mengalirkan daya. Kabel USB-A ke USB-C selalu berfungsi.
- WiFi menambah arus sekitar 0,1 A. Firmware menurunkan daya pancar WiFi agar lonjakan arus dan panas tetap kecil; jarak beberapa meter ke ponsel tetap terjangkau.
- Jika ESP32 mulai ulang saat servo bergerak, aplikasi menampilkan **mulai ulang karena tegangan turun** di menu Diagnostik. Pastikan kapasitor terpasang dan power bank ≥ 2 A. Port USB laptop hanya mampu 0,5–0,9 A, jadi gunakan power bank saat menguji servo.
- Tombol BOOT ESP32 berada di dalam kotak dan biasanya tidak diperlukan. Jika pengunggahan terus gagal, angkat G_EspCover, tarik papan keluar, lalu tahan BOOT sambil mencolokkan kabel.

## Uji bertahap
1. **Unggah firmware** (lihat bagian Firmware), lalu hubungkan aplikasi (lihat bagian Aplikasi).
2. **Pasang horn pada posisi tengah.** Pilih mode **Tengah** di aplikasi, lalu pasang horn kedua servo sehingga rangka dan kamera lurus. Sisa kemiringan kecil diluruskan melalui menu Kalibrasi, bagian *Posisi tengah servo*.
3. **Atur servo (tanpa kalibrasi servo).** Di menu Kalibrasi → Servo, biarkan kecepatan bawaan 60°/s. Pilih mode Stabil lalu miringkan dudukan pelan-pelan: kamera harus melawan kemiringan. Jika malah ikut miring makin jauh, ubah arah servo sumbu itu. Batas atas-bawah bawaan 16° menunduk dan 1° mendongak supaya gigi servo tidak selip. Penjelasan angka sudut ada di README, bagian *Batas servo dan konversi sudut*.
4. **Kalibrasi datar.** Letakkan waterpass atau ponsel (aplikasi waterpass) di atas D435, masukkan angka yang terbaca, lalu simpan. Setelah itu, dalam mode Tengah dengan badan tegak, tekan **Samakan model 3D** agar digital twin sama dengan gimbal Anda.
5. **Uji dengan tangan.** Miringkan dudukan perlahan: kamera harus tetap pada sudut target. Grafik di menu Kontrol menunjukkan sisa selisihnya.
6. **Jika bergetar atau berayun**, turunkan P atau I di menu PID sedikit demi sedikit, lalu jalankan **Uji respons**. Hasil yang baik: kamera mencapai target dalam waktu kurang dari ±150 ms dan melewati target kurang dari ±15%. Jika servo berdengung saat diam, naikkan *Zona mati servo* atau *Dead band*, atau turunkan *Denyut servo* ke 50 Hz.
7. **Pasang di sabuk dan uji sambil berjalan.** Program depth di komputer dapat membaca baris `T,...` dari USB untuk mengoreksi sisa kemiringan yang tidak sempat dikejar servo.

## Firmware (`firmware/D435_gimbal_ESP32C3/D435_gimbal_ESP32C3.ino`)
**Pengaturan Arduino IDE:**
- Boards Manager: **esp32 by Espressif Systems** versi 3.x.
- Tools → Board: **ESP32C3 Dev Module**.
- Tools → USB CDC On Boot: **Enabled**.
- Tools → Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**. Partisi ini diperlukan karena aplikasi web ikut tertanam di firmware.
- Berkas `web_files.h` harus berada di folder yang sama dengan berkas `.ino`.
- Tidak diperlukan pustaka tambahan.

Jika aplikasi di `docs/` diubah, jalankan `python3 tools/embed_web.py` sebelum mengunggah agar perubahan ikut masuk ke firmware.

## Aplikasi pengatur
**Menghubungkan lewat WiFi:**
1. Aktifkan hotspot ponsel, lalu nyalakan gimbal. Gimbal akan terhubung sendiri ke hotspot yang sudah dikenalnya.
2. Pada ponsel atau iPad yang terhubung ke hotspot tersebut, buka **http://gimbal-d435.local** di Safari atau Chrome. Aplikasi langsung dimuat dari gimbal dan meminta sandi.
3. Jika alamat `.local` tidak dapat dibuka (terjadi pada sebagian ponsel Android), gunakan alamat IP yang tertera di menu **Perangkat → WiFi** atau di Serial Monitor.

**Pemakaian pertama** (gimbal belum mengenal hotspot apa pun):
1. Gimbal memancarkan WiFi sendiri bernama **Gimbal-D435-xxxx** dengan sandi **gimbald435**. Empat karakter terakhir berasal dari chip, sehingga dua gimbal tidak akan memakai nama yang sama.
2. Hubungkan ponsel ke WiFi tersebut, lalu buka **http://192.168.4.1**.
3. Pada menu **Perangkat → WiFi**, tekan **Cari hotspot**, pilih hotspot ponsel Anda, masukkan sandinya, lalu tekan **Hubungkan gimbal**.
4. Aplikasi menampilkan alamat baru gimbal. Hubungkan ponsel atau iPad ke hotspot yang sama, lalu buka alamat tersebut. Tekan **Selesai, matikan WiFi gimbal**, atau tunggu dua menit hingga WiFi milik gimbal nonaktif dengan sendirinya.

Apabila hotspot tidak ditemukan selama 20 detik, WiFi milik gimbal aktif kembali agar pengaturan tetap dapat diubah.

**Hal-hal penting:**
- Ponsel yang memancarkan hotspot tetap dapat memakai data seluler.
- Hingga tiga ponsel atau iPad dapat terhubung bersamaan. Semuanya menerima data yang sama, perubahan dari satu perangkat langsung terlihat di perangkat lain, dan aplikasi memberi tahu apabila ada perangkat lain yang terhubung.
- **Sandi aplikasi bawaan: 123456.** Gantilah di menu Perangkat. Sandi tidak pernah dikirim: gimbal mengirim bilangan acak, lalu aplikasi menjawab dengan sidik SHA-256 dari bilangan tersebut dan sandi. Setelah lima kali salah, koneksi dikunci selama 30 detik.
- **Tanda sudah terhubung:** status hijau di aplikasi, bunyi dan getar pada ponsel, LED biru ESP32 menyala terus, dan kamera mengangguk kecil satu kali.
- **Lupa sandi:** hubungkan kabel USB ke komputer, buka aplikasi di Chrome atau Edge (GitHub Pages, tombol *Hubungkan lewat USB*; koneksi USB tidak memerlukan sandi), lalu buat sandi baru.
- **Koneksi USB dan mode demo** tersedia di https://khaichi11.github.io/stabilizer_camera/.

**Menu aplikasi:**

| Menu | Isi |
|---|---|
| Kontrol | Sudut atas-bawah (25/30/35/40/45° atau nilai bebas 10–60°), sudut kanan-kiri, mode Stabil / Tengah / Manual / Nonaktif, grafik 10 detik |
| Kalibrasi | Pengaturan servo (kecepatan, arah, kabel, skala, batas, konversi sudut), kalibrasi datar, kalibrasi gyro, samakan model 3D, posisi tengah servo |
| PID | P, I, D, peredaman gerak badan, prediksi, jeda dan waktu tanggap servo, zona mati servo, kompensasi zona mati, dead band, penghalus, kecepatan servo, denyut servo, filter akselerometer, filter sensor, uji respons dan CSV |
| Diagnostik | Status perangkat, penyebab mulai ulang, log kejadian, laporan debug, perekaman 30 detik, terminal perintah |
| Perangkat | Skema rangkaian, WiFi, sandi, nama gimbal (sekaligus alamatnya), mode saat dinyalakan, LED, bunyi |

## Cara kerja stabilisasi
IMU menempel pada kamera, sehingga yang diukur adalah orientasi kamera itu sendiri. Servo MG90S menahan sudut apa pun yang diperintahkan, sehingga pengendali menentukan **sudut servo** melalui empat bagian berikut.

1. **Pembacaan sudut.** Arah gravitasi dihitung sebagai vektor dalam sumbu kamera. Gyro memutar vektor ini setiap 5 ms, sedangkan akselerometer mengoreksinya secara perlahan (konstanta waktu 0,6 s saat diam). Ketika langkah kaki terdeteksi dari pantulan naik-turun badan, akselerometer diberi bobot lebih kecil (3 s) tetapi merata pada setiap bagian langkah, sehingga dorongan maju-mundur saat melangkah tidak terbaca sebagai kemiringan.
2. **Peredaman gerak badan (umpan maju).** Firmware memiliki model servo: perintah yang dikirim, ditunda sebesar jeda servo, lalu diperlambat sebesar waktu tanggap servo. Gerak badan dihitung sebagai sudut kamera dikurangi model tersebut. Servo langsung diperintah melawan gerak badan, ditambah prediksi untuk menutupi jeda servo. Jeda dan waktu tanggap servo diukur saat kalibrasi otomatis.
3. **PID** mengoreksi sisa kesalahan model (skala servo, posisi horn, beban) dan menjaga kamera tepat pada target:
   - **I** menggeser servo hingga kamera tepat pada target tanpa sisa;
   - **P** menambah reaksi langsung;
   - **D** memakai kecepatan putar dari gyro untuk meredam guncangan.
4. **Gerak servo yang halus.** Servo hanya membaca perintah baru pada setiap denyut. Pada 50 Hz, servo bergerak patah-patah setiap 20 ms dan menambah getaran cepat pada kamera (efek *jello* pada video RGB). Firmware memakai 100 Hz sebagai bawaan, yang masih aman untuk servo analog seperti MG90S. Model servo juga memperhitungkan zona mati servo, sehingga pengendali tidak memaksa servo bergerak bolak-balik dan servo tidak berdengung saat diam.

Nilai bawaan diuji di simulasi fisik dengan pola jalan realistis: sisa guncangan kamera sekitar 0,4° RMS untuk sudut atas-bawah dan 0,2–0,3° RMS untuk sudut kanan-kiri, sedangkan tanpa gimbal guncangannya 6–10°. Pengujian yang sama juga dilakukan untuk servo yang lebih lambat, zona mati yang lebih besar, gigi yang longgar, menaiki tangga, berjalan cepat, dan berjalan pincang.

Gimbal dua sumbu tidak dapat meredam putaran mendatar (*yaw*) kamera yang terjadi ketika pinggang berputar.

**Perintah teks** (Serial Monitor 115200 atau menu terminal di aplikasi). Kirim `HELP` untuk daftar lengkap.

| Perintah | Fungsi |
|---|---|
| `SET pitch 30` | Mengatur sudut atas-bawah 30° (semua pengaturan lain juga melalui `SET`) |
| `SET pwmhz 50` | Mengatur denyut servo (50–333 Hz) |
| `MODE stab` / `center` / `manual` / `off` | Memilih mode |
| `CAL gyro` / `CAL level 35.0 0` / `CAL twin` | Kalibrasi gyro, datar, dan model 3D |
| `STEP p 5` | Uji respons sudut atas-bawah 5° |
| `GET` / `STATUS` / `EVLOG` | Menampilkan pengaturan, status, dan log kejadian |
| `WIFI` / `WIFI SCAN` | Status WiFi / mencari hotspot |
| `WIFI FORGET` | Melupakan hotspot dan mengaktifkan WiFi milik gimbal |
| `WIFI OFF` / `WIFI ON` | Menonaktifkan atau mengaktifkan WiFi (hanya lewat USB) |
| `PIN 654321` | Mengganti sandi aplikasi |

**Data sudut untuk komputer:** baris `T,waktu,atas_bawah,kanan_kiri,target_atas_bawah,target_kanan_kiri,servo_atas_bawah,servo_kanan_kiri,laju_atas_bawah,laju_kanan_kiri,mode,flags` dikirim 20 kali per detik (dapat diatur di menu Perangkat). Bit 128 pada `flags` berarti langkah kaki sedang terdeteksi.

## Agar putaran lancar (penting untuk stabilisasi)
- **Kedua poros harus segaris.** Poros kiri adalah as servo putar atas-bawah, sedangkan poros kanan adalah tabung rangka (G_PitchFrame) di bantalan G_Base. Keduanya sudah segaris dalam desain. Jangan memaksa rangka masuk apabila terasa seret; amplas tipis bagian tabung atau bantalannya.
- **Kelonggaran bantalan 0,25 mm.** Setelah dicetak, putar rangka dengan tangan tanpa servo terpasang. Rangka harus turun sendiri karena beratnya, tanpa tersangkut.
- **Oleskan tipis** vaselin atau gemuk silikon pada tabung. Jangan memakai oli encer, karena PLA/PETG dapat menjadi lengket dan berdebu.
- **Kabel tidak boleh menahan gerak.** Kabel yang melewati tabung berada tepat di sumbu putar, sehingga hampir tidak tertekuk. Lengkungan kabel IMU (dudukan → rangka) dan kabel USB kamera harus longgar.
- **Kamera di tengah dudukan.** Penjepit kamera membuat titik berat berada dekat sumbu kanan-kiri. Beban servo kecil, sehingga gerakannya halus dan tidak bergetar.
- Jika servo terdengar berdengung saat diam, ikuti langkah 6 pada bagian Uji bertahap.

## Batas gerak (sudah diperiksa di Fusion)
- Sudut atas-bawah 10–60° dan sudut kanan-kiri ±25°: tidak ada bagian yang bertabrakan, termasuk dengan kotak ESP32 dan kepala colokan daya.
- Kotak ESP32 menyatu dengan dinding depan klip di sepanjang lebar dan tinggi bidang belakangnya, bukan hanya menempel di ujung pelat.
- **Jalur pemasangan juga sudah diperiksa:**
  - rangka diturunkan dari atas ke bantalan tanpa menabrak dudukan maupun servo;
  - G_PivotCap digeser masuk tanpa menabrak apa pun dan tanpa menjepit kabel;
  - G_EspCover diturunkan 30 mm tanpa menabrak kabel dari poros.
- Saluran kabel, penutup poros, kenop sabuk, dan pengunci kabel kamera sudah dipastikan tidak tersentuh dudukan kamera di semua posisi gerak.
- Sudut pandang depth 87 × 58° tidak terhalang di semua posisi tersebut.
