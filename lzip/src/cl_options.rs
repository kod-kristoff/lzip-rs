#[derive(Debug, Copy, Clone)]
pub struct ClOptions {
    pub ignore_empty: bool,
    pub ignore_marking: bool,
    pub ignore_trailing: bool,
    pub loose_trailing: bool,
}

impl Default for ClOptions {
    fn default() -> Self {
        Self {
            ignore_empty: true,
            ignore_marking: true,
            ignore_trailing: true,
            loose_trailing: false,
        }
    }
}
