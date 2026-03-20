//! Expression engine — formula binding and evaluation.
//!
//! Provides a simple expression language that can reference property
//! values and perform arithmetic.  This is Phase C6 and will be
//! expanded later.

use std::collections::HashMap;

// ── AST ────────────────────────────────────────────────────────────

/// Expression AST node.
#[derive(Debug, Clone)]
pub enum Expr {
    /// Numeric literal.
    Number(f64),
    /// String literal.
    Str(String),
    /// Boolean literal.
    Bool(bool),
    /// Binary operation.
    BinaryOp { op: BinOp, lhs: Box<Expr>, rhs: Box<Expr> },
    /// Unary operation.
    UnaryOp { op: UnOp, operand: Box<Expr> },
    /// Function call.
    FuncCall { name: String, args: Vec<Expr> },
    /// Property reference: object_name.property_name
    PropRef { object: String, property: String },
    /// Conditional: if(cond, then, else)
    Conditional { cond: Box<Expr>, then_expr: Box<Expr>, else_expr: Box<Expr> },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinOp {
    Add, Sub, Mul, Div, Pow, Mod,
    Eq, Ne, Lt, Gt, Le, Ge,
    And, Or,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnOp {
    Neg, Not,
}

// ── Evaluation ─────────────────────────────────────────────────────

/// Context for resolving property references during evaluation.
pub trait EvalContext {
    fn resolve_property(&self, object: &str, property: &str) -> Option<f64>;
}

/// Evaluate an expression to a floating-point result.
pub fn eval(expr: &Expr, ctx: &dyn EvalContext) -> Result<f64, String> {
    match expr {
        Expr::Number(n) => Ok(*n),
        Expr::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        Expr::Str(_) => Err("Cannot evaluate string as number".into()),
        Expr::BinaryOp { op, lhs, rhs } => {
            let l = eval(lhs, ctx)?;
            let r = eval(rhs, ctx)?;
            Ok(match op {
                BinOp::Add => l + r,
                BinOp::Sub => l - r,
                BinOp::Mul => l * r,
                BinOp::Div => {
                    if r.abs() < 1e-300 { return Err("Division by zero".into()); }
                    l / r
                }
                BinOp::Pow => l.powf(r),
                BinOp::Mod => {
                    if r.abs() < 1e-300 { return Err("Modulo by zero".into()); }
                    l % r
                }
                BinOp::Eq => if (l - r).abs() < 1e-12 { 1.0 } else { 0.0 },
                BinOp::Ne => if (l - r).abs() >= 1e-12 { 1.0 } else { 0.0 },
                BinOp::Lt => if l < r { 1.0 } else { 0.0 },
                BinOp::Gt => if l > r { 1.0 } else { 0.0 },
                BinOp::Le => if l <= r { 1.0 } else { 0.0 },
                BinOp::Ge => if l >= r { 1.0 } else { 0.0 },
                BinOp::And => if l != 0.0 && r != 0.0 { 1.0 } else { 0.0 },
                BinOp::Or => if l != 0.0 || r != 0.0 { 1.0 } else { 0.0 },
            })
        }
        Expr::UnaryOp { op, operand } => {
            let v = eval(operand, ctx)?;
            Ok(match op {
                UnOp::Neg => -v,
                UnOp::Not => if v == 0.0 { 1.0 } else { 0.0 },
            })
        }
        Expr::FuncCall { name, args } => {
            let vals: Vec<f64> = args.iter().map(|a| eval(a, ctx)).collect::<Result<_, _>>()?;
            match name.as_str() {
                "sin" => Ok(vals.first().copied().unwrap_or(0.0).sin()),
                "cos" => Ok(vals.first().copied().unwrap_or(0.0).cos()),
                "tan" => Ok(vals.first().copied().unwrap_or(0.0).tan()),
                "asin" => Ok(vals.first().copied().unwrap_or(0.0).asin()),
                "acos" => Ok(vals.first().copied().unwrap_or(0.0).acos()),
                "atan" => Ok(vals.first().copied().unwrap_or(0.0).atan()),
                "atan2" => Ok(vals.first().copied().unwrap_or(0.0).atan2(vals.get(1).copied().unwrap_or(0.0))),
                "sqrt" => Ok(vals.first().copied().unwrap_or(0.0).sqrt()),
                "abs" => Ok(vals.first().copied().unwrap_or(0.0).abs()),
                "exp" => Ok(vals.first().copied().unwrap_or(0.0).exp()),
                "log" => Ok(vals.first().copied().unwrap_or(0.0).ln()),
                "ceil" => Ok(vals.first().copied().unwrap_or(0.0).ceil()),
                "floor" => Ok(vals.first().copied().unwrap_or(0.0).floor()),
                "round" => Ok(vals.first().copied().unwrap_or(0.0).round()),
                "min" => Ok(vals.iter().copied().fold(f64::INFINITY, f64::min)),
                "max" => Ok(vals.iter().copied().fold(f64::NEG_INFINITY, f64::max)),
                "pow" => Ok(vals.first().copied().unwrap_or(0.0).powf(vals.get(1).copied().unwrap_or(0.0))),
                "hypot" => Ok(vals.first().copied().unwrap_or(0.0).hypot(vals.get(1).copied().unwrap_or(0.0))),
                "pi" => Ok(std::f64::consts::PI),
                "e" => Ok(std::f64::consts::E),
                _ => Err(format!("Unknown function: {name}")),
            }
        }
        Expr::PropRef { object, property } => {
            ctx.resolve_property(object, property)
                .ok_or_else(|| format!("Cannot resolve {object}.{property}"))
        }
        Expr::Conditional { cond, then_expr, else_expr } => {
            let c = eval(cond, ctx)?;
            if c != 0.0 { eval(then_expr, ctx) } else { eval(else_expr, ctx) }
        }
    }
}

// ── Simple parser ──────────────────────────────────────────────────

/// Parse a simple expression string.
/// Supports: numbers, +, -, *, /, ^, parentheses, function calls,
/// and property references (Object.Property).
pub fn parse(input: &str) -> Result<Expr, String> {
    let tokens = tokenize(input)?;
    let mut pos = 0;
    let result = parse_expr(&tokens, &mut pos)?;
    if pos < tokens.len() {
        return Err(format!("Unexpected token at position {pos}: {:?}", tokens[pos]));
    }
    Ok(result)
}

#[derive(Debug, Clone)]
enum Token {
    Number(f64),
    Ident(String),
    Op(char),
    LParen,
    RParen,
    Comma,
    Dot,
}

fn tokenize(input: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let chars: Vec<char> = input.chars().collect();
    let mut i = 0;
    while i < chars.len() {
        match chars[i] {
            c if c.is_whitespace() => { i += 1; }
            '(' => { tokens.push(Token::LParen); i += 1; }
            ')' => { tokens.push(Token::RParen); i += 1; }
            ',' => { tokens.push(Token::Comma); i += 1; }
            '.' => { tokens.push(Token::Dot); i += 1; }
            '+' | '-' | '*' | '/' | '^' | '%' => {
                tokens.push(Token::Op(chars[i]));
                i += 1;
            }
            c if c.is_ascii_digit() || c == '.' => {
                let start = i;
                while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                    i += 1;
                }
                // Handle scientific notation
                if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
                    i += 1;
                    if i < chars.len() && (chars[i] == '+' || chars[i] == '-') {
                        i += 1;
                    }
                    while i < chars.len() && chars[i].is_ascii_digit() {
                        i += 1;
                    }
                }
                let s: String = chars[start..i].iter().collect();
                let n: f64 = s.parse().map_err(|_| format!("Invalid number: {s}"))?;
                tokens.push(Token::Number(n));
            }
            c if c.is_ascii_alphabetic() || c == '_' => {
                let start = i;
                while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                    i += 1;
                }
                let s: String = chars[start..i].iter().collect();
                tokens.push(Token::Ident(s));
            }
            c => return Err(format!("Unexpected character: {c}")),
        }
    }
    Ok(tokens)
}

fn parse_expr(tokens: &[Token], pos: &mut usize) -> Result<Expr, String> {
    parse_additive(tokens, pos)
}

fn parse_additive(tokens: &[Token], pos: &mut usize) -> Result<Expr, String> {
    let mut left = parse_multiplicative(tokens, pos)?;
    while *pos < tokens.len() {
        match &tokens[*pos] {
            Token::Op('+') => { *pos += 1; let r = parse_multiplicative(tokens, pos)?; left = Expr::BinaryOp { op: BinOp::Add, lhs: Box::new(left), rhs: Box::new(r) }; }
            Token::Op('-') => { *pos += 1; let r = parse_multiplicative(tokens, pos)?; left = Expr::BinaryOp { op: BinOp::Sub, lhs: Box::new(left), rhs: Box::new(r) }; }
            _ => break,
        }
    }
    Ok(left)
}

fn parse_multiplicative(tokens: &[Token], pos: &mut usize) -> Result<Expr, String> {
    let mut left = parse_power(tokens, pos)?;
    while *pos < tokens.len() {
        match &tokens[*pos] {
            Token::Op('*') => { *pos += 1; let r = parse_power(tokens, pos)?; left = Expr::BinaryOp { op: BinOp::Mul, lhs: Box::new(left), rhs: Box::new(r) }; }
            Token::Op('/') => { *pos += 1; let r = parse_power(tokens, pos)?; left = Expr::BinaryOp { op: BinOp::Div, lhs: Box::new(left), rhs: Box::new(r) }; }
            Token::Op('%') => { *pos += 1; let r = parse_power(tokens, pos)?; left = Expr::BinaryOp { op: BinOp::Mod, lhs: Box::new(left), rhs: Box::new(r) }; }
            _ => break,
        }
    }
    Ok(left)
}

fn parse_power(tokens: &[Token], pos: &mut usize) -> Result<Expr, String> {
    let base = parse_unary(tokens, pos)?;
    if *pos < tokens.len() {
        if let Token::Op('^') = &tokens[*pos] {
            *pos += 1;
            let exp = parse_unary(tokens, pos)?;
            return Ok(Expr::BinaryOp { op: BinOp::Pow, lhs: Box::new(base), rhs: Box::new(exp) });
        }
    }
    Ok(base)
}

fn parse_unary(tokens: &[Token], pos: &mut usize) -> Result<Expr, String> {
    if *pos < tokens.len() {
        if let Token::Op('-') = &tokens[*pos] {
            *pos += 1;
            let operand = parse_primary(tokens, pos)?;
            return Ok(Expr::UnaryOp { op: UnOp::Neg, operand: Box::new(operand) });
        }
    }
    parse_primary(tokens, pos)
}

fn parse_primary(tokens: &[Token], pos: &mut usize) -> Result<Expr, String> {
    if *pos >= tokens.len() {
        return Err("Unexpected end of expression".into());
    }
    match &tokens[*pos] {
        Token::Number(n) => {
            let n = *n;
            *pos += 1;
            Ok(Expr::Number(n))
        }
        Token::LParen => {
            *pos += 1;
            let expr = parse_expr(tokens, pos)?;
            if *pos >= tokens.len() || !matches!(&tokens[*pos], Token::RParen) {
                return Err("Expected closing parenthesis".into());
            }
            *pos += 1;
            Ok(expr)
        }
        Token::Ident(name) => {
            let name = name.clone();
            *pos += 1;
            // Check for function call: ident(args...)
            if *pos < tokens.len() && matches!(&tokens[*pos], Token::LParen) {
                *pos += 1; // consume (
                let mut args = Vec::new();
                if *pos < tokens.len() && !matches!(&tokens[*pos], Token::RParen) {
                    args.push(parse_expr(tokens, pos)?);
                    while *pos < tokens.len() && matches!(&tokens[*pos], Token::Comma) {
                        *pos += 1;
                        args.push(parse_expr(tokens, pos)?);
                    }
                }
                if *pos >= tokens.len() || !matches!(&tokens[*pos], Token::RParen) {
                    return Err("Expected closing parenthesis in function call".into());
                }
                *pos += 1;
                return Ok(Expr::FuncCall { name, args });
            }
            // Check for property reference: ident.ident
            if *pos < tokens.len() && matches!(&tokens[*pos], Token::Dot) {
                *pos += 1;
                if *pos >= tokens.len() {
                    return Err("Expected property name after '.'".into());
                }
                if let Token::Ident(prop) = &tokens[*pos] {
                    let prop = prop.clone();
                    *pos += 1;
                    return Ok(Expr::PropRef { object: name, property: prop });
                }
                return Err("Expected property name after '.'".into());
            }
            // Known constants
            match name.as_str() {
                "pi" => Ok(Expr::Number(std::f64::consts::PI)),
                "e" => Ok(Expr::Number(std::f64::consts::E)),
                "true" => Ok(Expr::Bool(true)),
                "false" => Ok(Expr::Bool(false)),
                _ => Ok(Expr::PropRef { object: String::new(), property: name }),
            }
        }
        t => Err(format!("Unexpected token: {t:?}")),
    }
}

// ── ExpressionEngine ───────────────────────────────────────────────

/// Binds expressions to property paths and evaluates them.
#[derive(Debug, Clone, Default)]
pub struct ExpressionEngine {
    /// Map from "PropertyName" → expression text.
    bindings: HashMap<String, String>,
}

impl ExpressionEngine {
    pub fn new() -> Self { Self { bindings: HashMap::new() } }

    /// Bind an expression to a property.
    pub fn set_expression(&mut self, prop_name: &str, expr_text: &str) {
        self.bindings.insert(prop_name.to_string(), expr_text.to_string());
    }

    /// Remove an expression binding.
    pub fn remove_expression(&mut self, prop_name: &str) {
        self.bindings.remove(prop_name);
    }

    /// Get bound expression text.
    pub fn get_expression(&self, prop_name: &str) -> Option<&str> {
        self.bindings.get(prop_name).map(|s| s.as_str())
    }

    /// All bindings.
    pub fn bindings(&self) -> &HashMap<String, String> { &self.bindings }

    /// Evaluate all bound expressions and return their results.
    pub fn evaluate_all(&self, ctx: &dyn EvalContext) -> Vec<(String, Result<f64, String>)> {
        self.bindings.iter().map(|(name, text)| {
            let result = parse(text).and_then(|expr| eval(&expr, ctx));
            (name.clone(), result)
        }).collect()
    }

    /// Get object names referenced by all expressions (for dependency tracking).
    pub fn referenced_objects(&self) -> Vec<String> {
        let mut refs = Vec::new();
        for text in self.bindings.values() {
            if let Ok(expr) = parse(text) {
                collect_refs(&expr, &mut refs);
            }
        }
        refs.sort();
        refs.dedup();
        refs
    }

    pub fn is_empty(&self) -> bool { self.bindings.is_empty() }
    pub fn len(&self) -> usize { self.bindings.len() }
}

fn collect_refs(expr: &Expr, refs: &mut Vec<String>) {
    match expr {
        Expr::PropRef { object, .. } if !object.is_empty() => refs.push(object.clone()),
        Expr::BinaryOp { lhs, rhs, .. } => { collect_refs(lhs, refs); collect_refs(rhs, refs); }
        Expr::UnaryOp { operand, .. } => collect_refs(operand, refs),
        Expr::FuncCall { args, .. } => { for a in args { collect_refs(a, refs); } }
        Expr::Conditional { cond, then_expr, else_expr } => {
            collect_refs(cond, refs);
            collect_refs(then_expr, refs);
            collect_refs(else_expr, refs);
        }
        _ => {}
    }
}

// ── Tests ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    struct EmptyCtx;
    impl EvalContext for EmptyCtx {
        fn resolve_property(&self, _object: &str, _property: &str) -> Option<f64> { None }
    }

    struct TestCtx;
    impl EvalContext for TestCtx {
        fn resolve_property(&self, object: &str, property: &str) -> Option<f64> {
            match (object, property) {
                ("Box", "Width") => Some(100.0),
                ("Box", "Height") => Some(200.0),
                ("Sketch", "Length") => Some(50.0),
                _ => None,
            }
        }
    }

    #[test]
    fn parse_number() {
        let expr = parse("42").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), 42.0);
    }

    #[test]
    fn parse_arithmetic() {
        let expr = parse("2 + 3 * 4").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), 14.0);
    }

    #[test]
    fn parse_parentheses() {
        let expr = parse("(2 + 3) * 4").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), 20.0);
    }

    #[test]
    fn parse_unary_neg() {
        let expr = parse("-5 + 3").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), -2.0);
    }

    #[test]
    fn parse_power() {
        let expr = parse("2 ^ 10").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), 1024.0);
    }

    #[test]
    fn parse_function_call() {
        let expr = parse("sqrt(16)").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), 4.0);

        let expr = parse("max(3, 7, 1)").unwrap();
        assert_eq!(eval(&expr, &EmptyCtx).unwrap(), 7.0);
    }

    #[test]
    fn parse_constants() {
        let expr = parse("pi").unwrap();
        assert!((eval(&expr, &EmptyCtx).unwrap() - std::f64::consts::PI).abs() < 1e-12);
    }

    #[test]
    fn parse_property_ref() {
        let expr = parse("Box.Width + Box.Height").unwrap();
        assert_eq!(eval(&expr, &TestCtx).unwrap(), 300.0);
    }

    #[test]
    fn parse_complex_expression() {
        let expr = parse("Box.Width * 2 + sqrt(Box.Height)").unwrap();
        let result = eval(&expr, &TestCtx).unwrap();
        assert!((result - (200.0 + (200.0_f64).sqrt())).abs() < 1e-10);
    }

    #[test]
    fn expression_engine_basic() {
        let mut engine = ExpressionEngine::new();
        engine.set_expression("Depth", "Box.Width / 2");
        assert_eq!(engine.len(), 1);

        let results = engine.evaluate_all(&TestCtx);
        assert_eq!(results.len(), 1);
        let (name, val) = &results[0];
        assert_eq!(name, "Depth");
        assert!((val.as_ref().unwrap() - 50.0).abs() < 1e-10);
    }

    #[test]
    fn expression_engine_refs() {
        let mut engine = ExpressionEngine::new();
        engine.set_expression("Depth", "Box.Width + Sketch.Length");
        let refs = engine.referenced_objects();
        assert!(refs.contains(&"Box".to_string()));
        assert!(refs.contains(&"Sketch".to_string()));
    }

    #[test]
    fn division_by_zero() {
        let expr = parse("10 / 0").unwrap();
        assert!(eval(&expr, &EmptyCtx).is_err());
    }

    #[test]
    fn unresolved_property() {
        let expr = parse("Missing.Prop").unwrap();
        assert!(eval(&expr, &TestCtx).is_err());
    }
}
