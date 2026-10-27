"""Second experiment setup step, operating only on the frozen trial workspace."""
from pathlib import Path

root = Path(__file__).resolve().parents[1] / 'target/separator-variants-trial/source/swarm-core'
def edit(path, before, after, count=1):
    p=root/path
    t=p.read_text(encoding='utf-8')
    assert t.count(before)==count,(path,before[:60],t.count(before))
    p.write_text(t.replace(before,after),encoding='utf-8')

edit('src/eval/sep_evaluator.rs','use slotmap::SecondaryMap;', 'use slotmap::SecondaryMap;\nuse crate::util::bit_reversal_iterator::BitReversalIterator;')
edit('src/eval/sep_evaluator.rs','    n_evals: usize,','    n_evals: usize,\n    edge_order: Option<Vec<usize>>,')
edit('src/eval/sep_evaluator.rs','                    n_evals: 0,','                    n_evals: 0,\n                    edge_order: trial_edge_order(item),')
edit('src/eval/sep_evaluator.rs','            n_evals: 0,','            n_evals: 0,\n            edge_order: trial_edge_order(item),',2)
# The general indentation replacement also matches the pooled initializer.
edit('src/eval/sep_evaluator.rs','            edge_order: trial_edge_order(item),\n                    edge_order: trial_edge_order(item),','                    edge_order: trial_edge_order(item),')
edit('src/eval/sep_evaluator.rs', "impl<'a> SeparationEvaluator<'a> {", '''fn trial_edge_order(item:&Item)->Option<Vec<usize>> {
    (crate::trial_support::mode()==crate::trial_support::Mode::Kernel)
        .then(||BitReversalIterator::new(item.shape_cd.n_vertices()).collect())
}
impl<'a> SeparationEvaluator<'a> {''')
edit('src/eval/sep_evaluator.rs','(trial_mode==crate::trial_support::Mode::Kernel || trial_mode==crate::trial_support::Mode::Profile)','(trial_mode==crate::trial_support::Mode::Profile)')
edit('src/eval/sep_evaluator.rs','''        if trial_mode==crate::trial_support::Mode::Kernel && outside {
            crate::trial_support::count(&crate::trial_support::EARLY_REJECTS);
            return SampleEval::Invalid;
        }
''','')
edit('src/eval/sep_evaluator.rs','            &mut self.collector,','            &mut self.collector,\n            self.edge_order.as_deref(),')
edit('src/eval/specialized_jaguars_pipeline.rs','    collector: &mut SpecializedHazardCollector,','    collector: &mut SpecializedHazardCollector,\n    edge_order: Option<&[usize]>,')
edit('src/eval/specialized_jaguars_pipeline.rs','''    let custom_edge_iter = BitReversalIterator::new(shape.n_vertices()).map(|i| shape.edge(i));
    for edge in custom_edge_iter {
        v_quadtree.collect_collisions(&edge, collector);
        if collector.early_terminate(shape) {
            return;
        }
    }''','''    if let Some(indices)=edge_order {
        for &i in indices {
            let edge=shape.edge(i);
            v_quadtree.collect_collisions(&edge,collector);
            if collector.early_terminate(shape) { return; }
        }
    } else {
        let custom_edge_iter = BitReversalIterator::new(shape.n_vertices()).map(|i| shape.edge(i));
        for edge in custom_edge_iter {
            v_quadtree.collect_collisions(&edge, collector);
            if collector.early_terminate(shape) { return; }
        }
    }''')
edit('src/optimizer/separator.rs','    fn move_items_multi(', '    pub fn move_items_multi(')
edit('src/trial_support.rs','let _ = MODE.set(match s {','let _ = MODE.set(match s.trim_end_matches("_fixed").trim_end_matches("_checkempty") {')
edit('examples/async_separator_trial.rs','fn main() -> Result<()> {','''fn fixed(i:&SPInstance,p:SPProblem,workers:usize,seed:u64)->serde_json::Value {
    let mut sep=make_sep(i,p,seed,workers);
    let term=BasicTerminator::new();
    sep.change_strip_width(sep.prob.strip_width()*0.99,None);
    let start=Instant::now();
    let mut rows=Vec::new();
    for _ in 0..80 {
        if sep.ct.get_total_loss()==0.0 {sep.change_strip_width(sep.prob.strip_width()*0.995,None);}
        let stats=sep.move_items_multi(&term);
        let s=sep.prob.save();
        let mut poses=s.layout_snapshot.placed_items.values().map(|p|
            (p.item_id,p.d_transf.rotation(),p.d_transf.translation())).collect::<Vec<_>>();
        poses.sort_by_key(|p|p.0);
        rows.push(json!({"loss":sep.ct.get_total_loss(),"weighted_loss":sep.ct.get_total_weighted_loss(),
            "evals":stats.total_evals,"moves":stats.total_moves,"poses":poses}));
        sep.ct.update_weights();
    }
    json!({"rows":rows,"seconds":start.elapsed().as_secs_f64(),"counters":swarm_core::trial_support::snapshot()})
}
fn main() -> Result<()> {''')
edit('examples/async_separator_trial.rs','    let mut report=match a[4].as_str(){','''    if a[4].ends_with("_fixed") {return write(&a[3],&fixed(&i,p,workers,seed));}
    if a[4].ends_with("_checkempty") {
        let mut sep=make_sep(&i,p,seed,workers);
        let before=sep.prob.save();
        ensure!(sep.ct.get_total_loss()==0.0,"fixture must be clear");
        let (after,_)=sep.separate(&BasicTerminator::new(),&mut None);
        ensure!(before.strip_width()==after.strip_width(),"empty round changed width");
        for (key,p) in &before.layout_snapshot.placed_items {
            ensure!(p.d_transf==after.layout_snapshot.placed_items[key].d_transf,"empty round moved item");
        }
        certify(&i,&after)?;
        return write(&a[3],&json!({"certified":true,"counters":swarm_core::trial_support::snapshot()}));
    }
    let mut report=match a[4].as_str(){''')
print(root)
