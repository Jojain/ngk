//! Tunables for one STEP write.

/// What a STEP file says about itself.
///
/// None of this is geometry: it is the header and product-structure text that
/// a reader shows the user, plus the tolerance the document declares.
#[derive(Debug, Clone, PartialEq)]
pub struct StepWriteOptions {
    /// The product name, used for both `PRODUCT` and `FILE_NAME`.
    pub product_name: String,
    /// `FILE_DESCRIPTION`'s text.
    pub description: String,
    /// The author recorded in `FILE_NAME`.
    pub author: String,
    /// The organization recorded in `FILE_NAME`.
    pub organization: String,
    /// `FILE_NAME`'s timestamp, in ISO 8601.
    ///
    /// A plain string because the crate has no clock and no date dependency;
    /// a caller that wants a real timestamp supplies one. The default is the
    /// epoch, which is honest about being unset and keeps output of the same
    /// model byte-identical between runs.
    pub timestamp: String,
    /// The document's `UNCERTAINTY_MEASURE_WITH_UNIT`, in millimetres.
    ///
    /// Declares how close two positions must be for a reader to treat them as
    /// one. NGK's own `LINEAR_TOLERANCE` is three orders tighter than what
    /// files in the wild carry, and asserting that tightness in a file other
    /// kernels must re-stitch would be a claim about their arithmetic rather
    /// than ours — so this defaults to the `1e-7` such files use.
    pub uncertainty: f64,
}

impl Default for StepWriteOptions {
    fn default() -> Self {
        Self {
            product_name: "ngk".to_string(),
            description: "ngk model".to_string(),
            author: String::new(),
            organization: String::new(),
            timestamp: "1970-01-01T00:00:00".to_string(),
            uncertainty: 1.0e-7,
        }
    }
}

impl StepWriteOptions {
    /// Returns these options with `name` as the product name.
    pub fn named(name: impl Into<String>) -> Self {
        Self {
            product_name: name.into(),
            ..Self::default()
        }
    }
}

/// Tunables for one STEP read.
///
/// Every field here is a decision the exchange layer owns rather than one the
/// file dictates: what the file says is not in question, only what should be
/// done about it.
#[derive(Debug, Clone, PartialEq)]
pub struct StepReadOptions {
    /// Take the seam off a periodic face after building it.
    ///
    /// STEP writes a cylinder or sphere with its parameterization cut open
    /// along a seam, and that cut is not part of the shape. Import builds the
    /// seamed form exactly as the file states it and then, as a separate step
    /// this flag controls, runs `HealingOptions::seams_only()` over it.
    ///
    /// Turn it off to inspect precisely what the file said.
    pub heal_seams: bool,

    /// Fail on the first thing that cannot be carried across, rather than
    /// recording it and continuing.
    ///
    /// Off by default because real files contain faces that do not close, and
    /// aborting the whole file on one of them is useless in practice.
    /// On, it also promotes a `same_sense` disagreement from a report entry to
    /// an error.
    pub strict: bool,

    /// Override the document's own `UNCERTAINTY_MEASURE_WITH_UNIT`.
    ///
    /// `None` uses what the file declares, which is what a file written by
    /// another kernel means by "these two positions are the same". NGK's
    /// `LINEAR_TOLERANCE` is three orders tighter than the `1e-6`–`1e-7` such
    /// files carry, so this is what vertex merging, edge stitching and pcurve
    /// fitting are measured against.
    pub uncertainty: Option<f64>,
}

impl Default for StepReadOptions {
    fn default() -> Self {
        Self {
            heal_seams: true,
            strict: false,
            uncertainty: None,
        }
    }
}

impl StepReadOptions {
    /// Returns options that fail on anything they cannot carry across.
    pub fn strict() -> Self {
        Self {
            strict: true,
            ..Self::default()
        }
    }

    /// Returns options that build exactly what the file states, seam included.
    pub fn faithful() -> Self {
        Self {
            heal_seams: false,
            ..Self::default()
        }
    }
}
