# Prompt: Simulasi MATLAB Robot Penyiram Tanaman (Fuzzy-PID)

Tempel seluruh isi file ini ke sesi Claude Code baru (folder kerja: `c:\Asad\Kuliah\joki\King`),
atau ketik: **"Baca dan kerjakan simulasi_matlab/PROMPT_CLAUDE_CODE.md"**.

---

## Tugas

Buatkan **simulasi MATLAB** untuk penelitian berjudul
**"Inovasi Teknologi Robot Penyiram Tanaman Otomatis Menggunakan Metode Fuzzy-PID"**.
Simulasi harus meniru perilaku robot asli **sedekat mungkin**. Acuan perilaku robot asli adalah
firmware ESP32 di `program_robot_final_v10/program_robot_final_v10.ino`. Baca file itu dulu
sampai paham sebelum menulis kode MATLAB.

Dosen juga punya program simulasi atau format yang harus diikuti. Gambar, screenshot, atau
file dari dosen akan diberikan user.
- **Minta dulu gambar atau file dosen itu sebelum mulai menulis kode.** Baca semua teks di
  gambarnya.
- Format, nama variabel, dan tampilan grafik mengikuti dosen. Perilaku kendali mengikuti
  firmware.
- Kalau keduanya bertentangan, tanyakan ke user.

## Lingkungan

- Di PC ini terpasang **MATLAB R2024a** (`C:\Program Files\MATLAB\R2024a\bin`).
  Uji skrip dengan `matlab -batch "run('simulasi_matlab/main.m')"` sampai jalan tanpa error.
- Jangan bergantung pada Fuzzy Logic Toolbox untuk inti kendali. Tulis fuzzy Mamdani secara
  manual supaya hasilnya **identik** dengan firmware. Kalau toolbox tersedia, boleh ditambahkan
  file `.fis` terpisah hanya untuk gambar fungsi keanggotaan dan rule viewer di laporan.
- Semua file simulasi disimpan di folder `simulasi_matlab/`.
- User minta **setiap perubahan langsung di-commit dan di-push ke `origin main`**.

## Yang harus disalin persis dari firmware

Cari berdasarkan nama fungsi atau variabel, karena nomor baris bisa bergeser:

1. **Fuzzy Mamdani**, dari `MF_IN`, `OUT_CENTER`, `RULE_KP`, `RULE_KI`, `RULE_KD`,
   `trapmf()`, dan `fuzzyTuning()`:
   - Input e dan Δe masing-masing memakai 5 trapesium (NB, NS, ZE, PS, PB) pada semesta
     [-9, 9], disaturasi ke rentang itu.
   - Inferensi min–max dengan 25 aturan.
   - Defuzzifikasi COA titik pusat untuk Kp, Ki, Kd (7 label, ZE sampai VB, rentang 0–20).
   - Kalau tidak ada aturan yang aktif, nilai lama dipertahankan.
2. **PID diskrit**, dari `hitungPID()`:
   - Integral di-reset saat tanda e berganti.
   - Integral dibatasi ±`INTEGRAL_MAX` (0,5).
   - Rumus: u = Kp·e + Ki·I + Kd·Δe, disaturasi ±100.
   - Mode pembanding PID tetap memakai Kp = Ki = Kd = 9,25.
3. **Error dan setpoint**, dari bagian JELAJAH/MELAMBAT di `autoModeStep()`:
   - Setpoint tengah lorong: sp = (s_kiri + s_kanan)/2, lalu e = sp − s_kiri.
   - Mode satu sisi memakai `lastHalf`.
   - Sensor samping hanya dipakai bila < `g_navMax` (80 cm).
   - Ada zona mati `g_zonaMati` (2 cm).
4. **Pemetaan u → sudut kemudi**:
   - sudut = trunc(sat(g_steerSign · g_steerScale · u, ±g_autoMaxAngle)).
   - Perubahan sudut dibatasi `g_steerRate` derajat per siklus.
   - Nilai bawaan: g_steerSign = −1, g_steerScale = 0,08, g_autoMaxAngle = 10°,
     g_steerRate = 4°.
   - Stepper bergerak menuju target dengan kecepatan terbatas (`STEP_INTERVAL_US`, `g_stepsPerDeg`).
5. **Mesin keadaan navigasi**: JELAJAH, MELAMBAT, BERHENTI, BELOK_KIRI/KANAN, MUNDUR (3 fase),
   dan LIHAT.
   - Ambang bawaan: jarak aman 150 cm, belok 100 cm, mundur 15 cm, sisi terbuka 100 cm,
     belok paksa 40 cm, sudut belok 40°, lama belok 1500 ms, lama mundur 1500 ms.
   - PWM: jelajah 110, pelan 80, mundur 140.
   - Ikutkan juga filter sensor: median-3, penolakan lonjakan >15 cm, EMA α = 0,6, dan
     penahanan sensor depan saat tak terbaca.
6. **Penyiraman**: pompa ON bila sensor kiri atau kanan < `g_jarakTanaman` (40 cm), kecuali
   air habis.
7. **Waktu sampling**: kendali dihitung sekali per siklus sensor. Satu siklus membaca sensor
   berurutan dengan jeda 35 ms, jadi Ts ≈ 0,2–0,25 s. Gunakan Ts yang sama.

## Model yang harus dibuat

- **Robot Ackermann (model sepeda kinematik)**: x' = v·cosθ, y' = v·sinθ, θ' = v·tanδ / L.
  δ adalah sudut kemudi dari stepper. Parameter L (jarak sumbu roda), lebar robot (sekitar 60 cm),
  dan pemetaan PWM → kecepatan (m/s) dibuat sebagai variabel di satu file konfigurasi.
  **Tanyakan nilai sebenarnya ke user**, dan sementara pakai nilai wajar.
- **5 sensor ultrasonik**: depan, samping kiri, samping kanan, kiri (tanaman), kanan (tanaman).
  - Posisi dan arah sensor pada badan robot bisa diatur.
  - Jarak dihitung dengan ray-casting ke dinding atau tanaman, dengan sudut pancar ±15° (ambil
    jarak terdekat dalam kerucut).
  - Maksimum 400 cm. Di luar itu bernilai 999 (timeout), sama seperti `getDist()`.
  - Beri derau Gaussian kecil. Tambahkan opsi `anomali_dm556` yang membuat pembacaan semu
    sekitar 50 cm, meniru gangguan driver stepper pada robot asli.
  - Sensor level air tidak perlu disimulasikan (anggap air cukup), kecuali dosen meminta.
- **Lintasan**: lorong lurus selebar 100 cm dengan deretan tanaman di kiri dan kanan, lalu
  belokan atau dinding di ujung. Buat beberapa skenario: lurus, posisi awal tidak di tengah,
  belokan, dan rintangan di depan.

## Model motor DC dan pengereman

User meminta "DC injection braking". Teknik itu sebenarnya untuk motor AC induksi, sedangkan
motor penggerak robot adalah **motor DC sikat lewat driver BTS7960**. Karena itu yang
disimulasikan adalah padanannya untuk motor DC. Di laporan, sebut sebagai pengereman dinamis
(padanan DC injection braking pada motor DC).

- **Model motor DC** (ganti pemetaan PWM → kecepatan yang sederhana):
  - Listrik: V = R·i + L·di/dt + Ke·ω, dengan V = (PWM/255)·V_baterai.
  - Mekanik: J·dω/dt = Kt·i − b·ω − τ_beban.
  - τ_beban berasal dari massa robot, jari-jari roda, rasio gearbox, gesekan gelinding, dan
    tanjakan.
  - Semua parameter ada di `config_robot.m`. Tanyakan spesifikasi motor dan baterai ke user,
    dan sementara pakai nilai wajar untuk motor gearbox 12 V.
- **Tiga mode pengereman**, bisa dipilih:
  1. **Meluncur (coast)**: R_EN/L_EN LOW, sehingga arus motor ≈ 0 dan robot berhenti hanya
     karena gesekan.
  2. **Dynamic braking / REM**: kedua kaki motor dihubung-singkat lewat sisi bawah BTS7960
     (V = 0), sehingga back-EMF menghasilkan arus pengereman i = −Ke·ω/R. **Ini yang terjadi
     di robot asli** saat PWM 0 dengan enable ON, misalnya fitur REM lewat stik di mode
     otomatis atau keadaan BERHENTI/LIHAT.
  3. **Plugging**: tegangan dibalik (PWM negatif sesaat). Pengereman paling cepat, tetapi
     arusnya besar.
- **Output**:
  - grafik kecepatan, arus motor, dan jarak tempuh terhadap waktu untuk ketiga mode, dari
    kecepatan JELAJAH (PWM 110) dan dari kecepatan maksimum
  - tabel jarak berhenti, waktu berhenti, dan arus puncak
  - tandai bila arus puncak melewati batas proteksi BTS7960 (~43 A), karena proteksi itu
    mengunci driver
- **Dalam simulasi navigasi**: setiap kali mesin keadaan memberi PWM 0 (BERHENTI, LIHAT, jeda
  MUNDUR), gunakan mode dynamic braking supaya jarak berhenti robot realistis terhadap
  ambang `g_jarakMundur`. Uji juga skenario tanjakan: pada PWM 70% robot melambat atau
  berhenti, lalu fitur BOOST lewat stik dimodelkan sebagai kenaikan PWM sementara.

## Output yang diharapkan

1. **Animasi** robot bergerak di lorong: badan robot, roda kemudi, garis sinar sensor, tanaman
   yang menyala saat disiram.
2. **Grafik**: posisi lateral vs setpoint, e, Δe, Kp/Ki/Kd terhadap waktu, P/I/D/u, sudut
   kemudi, PWM, kelima jarak sensor, status pompa, dan status mesin keadaan.
3. **Perbandingan PID-Fuzzy vs PID tetap** pada skenario yang sama. Tampilkan dalam satu grafik
   dan tabel metrik: overshoot, settling time, steady-state error, IAE, ISE, RMSE.
4. **Ekspor log CSV** dengan header yang **sama persis** dengan robot (`LOG_HEADER` di firmware):
   `ms,run,metode,status,depan,s_kiri,s_kanan,kiri,kanan,setpoint,e,de,Kp,Ki,Kd,P,I,D,u,sudut,pwm,pompa,air_habis`.
   Dengan begitu hasil simulasi bisa dibandingkan langsung dengan log robot asli (unduhan
   `/log.csv`).
5. Skrip tambahan `bandingkan_log.m` yang membaca log robot asli dan log simulasi, lalu
   menggambar keduanya bertumpuk. Format log robot: Excel Indonesia, pemisah `;`, desimal `,`,
   baris pertama `sep=;`.
6. Gambar fungsi keanggotaan e, Δe, Kp, Ki, Kd dan permukaan (surface) Kp/Ki/Kd terhadap e dan
   Δe, untuk Bab 3/4 laporan.

## Struktur file yang disarankan

```
simulasi_matlab/
  main.m                 % pilih skenario & metode, jalankan, tampilkan
  config_robot.m         % semua parameter (dimensi, ambang, gain, Ts)
  fuzzy_mamdani.m        % salinan persis fuzzyTuning()
  pid_diskrit.m          % salinan persis hitungPID()
  kendali_otomatis.m     % salinan autoModeStep() (mesin keadaan + filter)
  model_ackermann.m
  model_motor_dc.m       % motor DC + mode pengereman (coast / dynamic braking / plugging)
  sensor_ultrasonik.m
  lintasan_*.m           % skenario
  animasi.m, plot_hasil.m, ekspor_log.m, bandingkan_log.m
  hasil/                 % gambar PNG & CSV keluaran
```

## Cara kerja

1. Baca firmware, lalu minta dan baca gambar atau file dosen.
2. Tanyakan parameter fisik yang belum diketahui: L, lebar dan panjang robot, posisi sensor,
   kecepatan pada PWM 80/110/140, dan lebar lorong sebenarnya.
3. Buat modul satu per satu.
4. Uji fuzzy: untuk beberapa pasang (e, Δe), hasil Kp/Ki/Kd harus sama dengan perhitungan
   tangan dari tabel rule. Contohnya e = 0 dan Δe = 0 harus menghasilkan Kp = 7,5, Ki = 11,
   Kd = 7,5, sama dengan nilai awal di firmware.
5. Jalankan semua skenario dengan `matlab -batch`, simpan gambar ke `hasil/`, lalu commit dan
   push.
6. Laporkan ke user dalam bahasa Indonesia: apa yang sudah jalan, hasil metrik, dan bagian mana
   yang masih memakai asumsi.
