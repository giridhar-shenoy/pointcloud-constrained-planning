# MORNING — pointcloud-constrained-planning (R2)

## WAITING FOR GIRI

### 1. GitHub Actions will not start on this private repo (billing)

- **What:** every CI run on `giridhar-shenoy/pointcloud-constrained-planning` ends in ~3 s with the
  annotation *"The job was not started because recent account payments have failed or your spending
  limit needs to be increased. Please check the 'Billing & plans' section in your settings"*
  (first seen on run 40593288166, 2026-10-28 03:44 IST).
- **Why it needs you:** the fix is either the account's billing / Actions spending limit, or making
  the repo public (public repos get free Actions minutes). Overnight rules forbid changing visibility,
  and billing is your account.
- **Recommendation:** make the repo public when you are happy with it (the original plumbing prompt
  planned `--public` anyway); otherwise fix the payment method or raise the spending limit. Then re-run
  the latest workflow with `gh run rerun <id>` or push any commit.
- **What it blocks:** the acceptance items "CI run on GitHub is green" (plumbing) and "CI green"
  (feature). Meanwhile I run the same build/lint/test/smoke commands natively on the laptop and log
  their output in CHECKPOINTS.md, and keep pushing only at checkpoints.

### 2. Perceptyne L3 row in the README job-description table

- **What:** the SPRINT OVERRIDE asks for a "How this maps to NUS 34489 / CMU NREC / Perceptyne L3"
  table. The NUS and CMU phrases are quoted in the feature prompt; no Perceptyne L3 wording was in
  my inputs, and I will not invent a JD quote.
- **Why it needs you:** only you have the JD text.
- **Recommendation:** paste the one phrase you most want to evidence (e.g. its motion-planning or
  perception line); the README row currently reads "TODO(Giri): paste the exact JD phrase".
- **What it blocks:** nothing else; one README row.
