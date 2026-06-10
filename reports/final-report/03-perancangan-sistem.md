# 3. Perancangan Sistem

## 3.1 Pemodelan Matematis Komponen Microgrid

### 3.1.1 Pemodelan PV

Panel surya (photovoltaic, PV) merupakan sumber energi terbarukan yang mengkonversi radiasi matahari menjadi energi listrik. Daya keluaran PV pada waktu $k$ dimodelkan sebagai fungsi dari iradiansi global pada bidang panel dan kapasitas terpasang sistem:

$$P_{pv}(k) = \frac{G(k)}{1000} \cdot P_{pv}^{\text{installed}} \cdot \eta_{\text{sys}} \quad [\text{kW}]$$

dengan:
- $G(k)$ = iradiansi global pada bidang panel [W/m²] pada jam $k$
- $P_{pv}^{\text{installed}}$ = kapasitas terpasang PV [kWp]
- $\eta_{\text{sys}}$ = efisiensi sistem yang mencakup rugi inverter, kabel, suhu, dan kotoran

Efisiensi sistem ditetapkan sebesar $\eta_{\text{sys}} = 0.85$ (85%), yang merupakan nilai tipikal untuk sistem PV grid-tied sesuai standar IEC 61724.

**Data iradiansi** yang digunakan dalam proyek ini bersumber dari PVGIS-ERA5 (Joint Research Centre, European Commission) untuk lokasi Depok, Jawa Barat (koordinat $-6.36^\circ$ LS, $106.83^\circ$ BT). Data yang digunakan adalah rata-rata tahunan iradiansi global per jam pada permukaan dengan kemiringan $15^\circ$ menghadap utara (optimal untuk lokasi ekuator). Gambar 3.1 menunjukkan profil rata-rata PV yang dihasilkan.

*[Tabel atau plot profil Ppv 24 jam]*

Dalam konteks pemodelan *Economic MPC*, PV diperlakukan sebagai **gangguan eksogen** (disturbance) yang nilainya sudah diketahui sebelumnya melalui prakiraan (*perfect forecast* pada v1). Nilai $P_{pv}(k)$ untuk seluruh horizon prediksi $k = 0, 1, \ldots, N_p-1$ disusun dalam vektor gangguan $d(k)$ bersama dengan beban dan harga listrik. Karena PV tidak mempengaruhi state SoC secara langsung, entri yang bersesuaian dalam matriks $E$ pada model state-space bernilai nol.

### 3.1.2 Pemodelan Baterai (BESS)

Battery Energy Storage System (BESS) dimodelkan sebagai akumulator energi dengan state tunggal: *State of Charge* (SoC). Model matematis yang digunakan adalah integrator murni:

$$x(k+1) = x(k) - \frac{\Delta T}{E_{\text{nom}}} \cdot u_{\text{batt}}(k)$$

dengan:
- $x(k) \in [0, 1]$ = SoC pada waktu $k$ (tak-berdimensi)
- $u_{\text{batt}}(k)$ = daya baterai [kW], positif = *discharging*, negatif = *charging*
- $\Delta T = 1$ jam = waktu sampling
- $E_{\text{nom}} = 5.0$ kWh = kapasitas energi nominal BESS

**Parameter operasi BESS:**

| Parameter | Nilai | Deskripsi |
|---|---|---|
| $E_{\text{nom}}$ | 5.0 kWh | Kapasitas energi nominal |
| $P_{\text{batt,max}}$ | 2.5 kW | Daya maksimum charge/discharge |
| $\text{SoC}_{\text{min}}$ | 0.2 (20%) | Batas bawah SoC |
| $\text{SoC}_{\text{max}}$ | 0.8 (80%) | Batas atas SoC |
| $x_0$ | 0.5 (50%) | SoC awal |
| $x_{\text{target}}$ | 0.5 (50%) | SoC target terminal |

Batas operasi $\text{SoC} \in [0.2, 0.8]$ dan $u_{\text{batt}} \in [-2.5, 2.5]$ kW diterapkan sebagai kendala *hard constraint* dalam optimasi QP.

**Penyederhanaan v1:** Efisiensi *round-trip* baterai dimodelkan 100% di dinamika (rugi-rugi dimasukkan ke biaya melalui regularisasi Tikhonov). Asumsi ini lazim ditemukan di literatur EMPC microgrid (Cortes-Aguirre et al., 2024) karena menjaga konveksitas QP.

### 3.1.3 Pemodelan Beban

Beban listrik dimodelkan sebagai profil permintaan daya yang bervariasi terhadap waktu. Dalam proyek ini, profil beban dinormalisasi terhadap beban puncak $P_{\text{load,peak}} = 4.0$ kW:

$$P_{\text{load}}(k) = p(k) \cdot P_{\text{load,peak}} \quad [\text{kW}]$$

dengan $p(k) \in [0, 1]$ adalah faktor beban per unit pada jam $k$.

Profil beban yang digunakan bersumber dari dataset **IEEE RTS-GMLC** (Reliability Test System - Grid Modernization Laboratory Consortium) untuk Region 1 pada tanggal 15 Juli (musim panas, beban puncak tahunan). Dataset ini merupakan prakiraan *day-ahead* dengan resolusi per jam yang telah dinormalisasi terhadap beban puncak sistem 2652.9 MW.

*[Tabel atau plot profil Pload 24 jam]*

Beban listrik, seperti PV, diperlakukan sebagai gangguan eksogen dalam formulasi MPC. Keseimbangan daya pada bus AC microgrid menghubungkan beban dengan PV, baterai, dan jaringan:

$$P_{\text{load}}(k) = P_{pv}(k) + u_{\text{batt}}(k) + u_{\text{grid}}(k)$$

Persamaan ini menjadi **kendala kesamaan** (equality constraint) dalam QP.

### 3.1.4 Model State-Space Lengkap

Seluruh komponen di atas dirangkum dalam model *discrete-time Linear Time-Invariant* (LTI) state-space:

**State:**
$$x(k) = \text{SoC}(k) \quad \in \mathbb{R}$$

**Control input:**
$$u(k) = \begin{bmatrix} u_{\text{grid}}(k) \\ u_{\text{batt}}(k) \end{bmatrix} \quad \in \mathbb{R}^2$$

**Gangguan (disturbance):**
$$d(k) = \begin{bmatrix} P_{pv}(k) \\ P_{\text{load}}(k) \\ c(k) \end{bmatrix} \quad \in \mathbb{R}^3$$

**Model dinamika (state update):**
$$x(k+1) = A x(k) + B u(k) + E d(k)$$

**Model keluaran:**
$$y(k) = C x(k) + D u(k) + F_e d(k)$$

dengan matriks-matriks:

$$A = 1, \quad B = \begin{bmatrix} 0 & -\frac{\Delta T}{E_{\text{nom}}} \end{bmatrix}, \quad E = \begin{bmatrix} 0 & 0 & 0 \end{bmatrix}$$

$$C = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad D = \begin{bmatrix} 1 & -1 \\ 0 & 0 \end{bmatrix}, \quad F_e = \begin{bmatrix} -1 & 1 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$

**Penjelasan matriks:**
- $A = 1$: SoC adalah integrator murni (tanpa self-discharge)
- $B = [0,\; -\Delta T/E_{\text{nom}}]$: hanya $u_{\text{batt}}$ yang mempengaruhi SoC; $u_{\text{grid}}$ tidak berdampak langsung ke state
- $E = [0,0,0]$: gangguan tidak mempengaruhi state secara langsung di v1
- Baris 1 $C$ dan $D$: keluaran pertama adalah $u_{\text{grid}}$ (dari keseimbangan daya: $u_{\text{grid}} = P_{\text{load}} - P_{pv} - u_{\text{batt}}$)
- Baris 2 $C$ dan $D$: keluaran kedua adalah SoC itu sendiri

Model ini diimplementasikan dalam fungsi `plant_model.m`.
