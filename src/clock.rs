// for f64 math
use num_traits::{Float, Num};

// nrx, nfx, no, nb
// clkr, clkf, clkod, bwad
#[allow(warnings)]
pub fn calculate_pll_register_value(freq_in: u32, freq_out: u32) -> Option<(u8, u8, u8, u8)> {
    /* constants */
    const vco_min: f64 = 3.5e+08;
    const vco_max: f64 = 1.75e+09;
    const ref_min: f64 = 1.36719e+07;
    const ref_max: f64 = 1.75e+09;
    const nr_min: i32 = 1;
    const nr_max: i32 = 16;
    const nf_min: i32 = 1;
    const nf_max: i32 = 64;
    const no_min: i32 = 1;
    const no_max: i32 = 16;
    const nb_min: i32 = 1;
    const nb_max: i32 = 64;
    const max_vco: i32 = 1;
    const ref_rng: i32 = 1;

    /* variables */
    let mut nr: i32 = 0;
    let mut nrx: i32 = 0;
    let mut nf: i32 = 0;
    let mut _nfi: i32 = 0;
    let mut no: i32 = 0;
    let mut noe: i32 = 0;
    let mut not: i32 = 0;
    let mut nor: i32 = 0;
    let mut nore: i32 = 0;
    let mut nb: i32 = 0;
    let mut first: i32 = 0;
    let mut firstx: i32 = 0;
    let mut found: i32 = 0;

    let mut nfx: i64 = 0;
    let (mut fin, mut fout, mut fvco) = (0_f64, 0_f64, 0_f64);
    let (mut val, mut nval, mut err, mut merr, mut terr) = (0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    let (mut x_nrx, mut x_no, mut x_nb) = (0_i32, 0_i32, 0_i32);
    let mut x_nfx: i64 = 0;
    let (mut x_fvco, mut x_err) = (0_f64, 0_f64);

    fin = freq_in as f64;
    fout = freq_out as f64;
    val = fout / fin;
    terr = 0.5 / ((nf_max / 2) as f64);
    first = 1;
    firstx = 1;

    if terr != -2.0 {
        first = 0;
        if terr == 0.0 {
            terr = 1e-16;
        }
        merr = fabs(terr);
    }
    found = 0;
    for nfi in val as i32..nf_max {
        nr = rint(nfi as f64 / val) as i32;
        if nr == 0 {
            continue;
        }
        if ref_rng != 0 && nr < nr_min {
            continue;
        }
        if fin / nr as f64 > ref_max {
            continue;
        }
        nrx = nr;
        nf = nfi;
        nfx = nfi as i64;
        nval = nfx as f64 / nr as f64;
        if nf == 0 {
            nf = 1;
        }
        err = 1_f64 - nval / val;

        if first != 0 || fabs(err) < merr * (1.0 + 1e-6) || fabs(err) < 1e-16 {
            not = (vco_max / fout).floor() as i32;
            // a for loop
            no = i32::min(no_max, not);
            loop {
                if no > no_min {
                    if ref_rng != 0 && (nr / no < nr_min) {
                        no -= 1;
                        continue;
                    }
                    if nr % no == 0 {
                        break;
                    }
                } else {
                    break;
                }
                no -= 1;
            }
            if nr % no != 0 {
                continue;
            }

            nor = if not > no_max { no_max / no } else { not / no };
            nore = nf_max / nf;
            if nor > nore {
                nor = nore;
            }
            noe = (vco_min / fout).ceil() as i32;
            if max_vco == 0 {
                nore = (noe - 1) / no + 1;
                nor = nore;
                not = 0; /* force next if to fail */
            }
            if (((no * nor) < (not >> 1)) || ((no * nor) < noe)) && ((no * nor) < (nf_max / nf)) {
                no = nf_max / nf;
                if no > no_max {
                    no = no_max;
                }
                if no > not {
                    no = not;
                }
                nfx *= no as i64;
                nf *= no;
                if (no > 1) && (firstx == 0) {
                    continue;
                }
            /* wait for larger nf in later iterations */
            } else {
                nrx /= no;
                nfx *= nor as i64;
                nf *= nor;
                no *= nor;
                if no > no_max {
                    continue;
                }
                if (nor > 1) && (firstx == 0) {
                    continue;
                }
                /* wait for larger nf in later iterations */
            }

            nb = nfx as i32;
            if nb < nb_min {
                nb = nb_min;
            }
            if nb > nb_max {
                continue;
            }

            fvco = fin / (nrx as f64) * (nfx as f64);
            if fvco < vco_min {
                continue;
            }
            if fvco > vco_max {
                continue;
            }
            if nf < nf_min {
                continue;
            }
            if (ref_rng != 0) && (fin / (nrx as f64) < ref_min) {
                continue;
            }
            if (ref_rng != 0) && (nrx > nr_max) {
                continue;
            }
            if !(((firstx != 0) && (terr < 0.0))
                || (fabs(err) < merr * (1.0 - 1e-6))
                || ((max_vco != 0) && (no > x_no)))
            {
                continue;
            }
            if (firstx == 0) && (terr >= 0.0) && (nrx > x_nrx) {
                continue;
            }

            found = 1;
            x_no = no;
            x_nrx = nrx;
            x_nfx = nfx;
            x_nb = nb;
            x_fvco = fvco;
            x_err = err;
            first = 0;
            firstx = 0;
            merr = fabs(err);
            if terr != -1.0 {
                continue;
            }
        }
    }
    if found == 0 {
        return None;
    }
    nrx = x_nrx;
    nfx = x_nfx;
    no = x_no;
    nb = x_nb;
    fvco = x_fvco;
    err = x_err;
    if (terr != -2.0) && (fabs(err) >= terr * (1.0 - 1e-6)) {
        return None;
    }

    Some((nrx as u8, nfx as u8, no as u8, nb as u8))
}

fn fabs(a: f64) -> f64 {
    if a >= 0.0 {
        a
    } else {
        -a
    }
}

fn rint(a: f64) -> f64 {
    if a >= 0.0 {
        (a + 0.5) as i32 as f64
    } else {
        (a - 0.5) as i32 as f64
    }
}
