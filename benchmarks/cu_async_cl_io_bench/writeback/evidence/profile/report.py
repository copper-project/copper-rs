"""Summarize each completed run. Spatial runs provide dirty-page high water and are marked separately."""
import collections, csv, gzip, json, math, pathlib, re, statistics, sys
root=pathlib.Path(sys.argv[1])
page=int(re.search(r'^([0-9]+)$',(root/'environment.txt').read_text(),re.M)[1])
def stats(values):
    if not values: return {}
    values=sorted(values)
    return {'mean':statistics.mean(values),**{f'p{p}':values[math.ceil(len(values)*p/100)-1] for p in [50,95,99]},'max':values[-1]}
all_runs={}
for ok in sorted(root.glob('*.ok')):
    key=ok.stem
    with gzip.open(root/(key+'.csv.gz'),'rt') as f:
        es=list(csv.DictReader(f))
    for e in es:
        for name in ['begin_ns','end_ns','slab','offset','bytes']:e[name]=int(e[name])
        e['ns']=e['end_ns']-e['begin_ns']
    es.sort(key=lambda e:e['begin_ns'])
    types=collections.defaultdict(list)
    for e in es:types[e['kind']].append(e)
    ticks=types['tick']; missed=[e for e in ticks if e['ns']>50_000_000]
    flush=types['flush']
    pages=set();last={};order=collections.Counter();runs=[];run=0;lastslab=None;switch=0
    for f in flush:
        s,b,n=f['slab'],f['offset'],f['bytes']
        pages.update((s,p) for p in range(b//page,(b+n-1)//page+1))
        prior=last.get(s)
        if prior:
            a,z=prior
            category='adjacent' if b==z else 'forward_gap' if b>z else 'backward' if b+n<=a else 'overlap'
            order[category]+=1
        if lastslab==s and prior and b==prior[1]:run+=n
        else:
            if run:runs.append(run)
            run=n
        switch+=lastslab is not None and lastslab!=s
        lastslab=s;last[s]=(b,b+n)
    if run:runs.append(run)
    # Count all completion-order inversions; the common serial path contributes zero.
    ends=[];inversions=0
    import bisect
    for e in flush:
        i=bisect.bisect_right(ends,e['end_ns'])
        inversions+=len(ends)-i
        ends.insert(i,e['end_ns'])
    data={'revision':ok.read_text().strip(),'times_ns':{k:stats([e['ns'] for e in types[k]]) for k in ['submission','record','flush','tick','final_drain']},
          'msync':{'count':len(flush),'request_bytes':stats([e['bytes'] for e in flush]),'total_bytes':sum(e['bytes'] for e in flush),'distinct_pages':len(pages),'order':dict(order),'slab_switches':switch,'mean_contiguous_run_bytes':statistics.mean(runs) if runs else 0,'completion_inversions':inversions},'over_50ms':len(missed),'longest_flushes':[{**e,'overlapping_missed_ticks':sum(t['begin_ns']<e['end_ns'] and t['end_ns']>e['begin_ns'] for t in missed)} for e in sorted(flush,key=lambda e:-e['ns'])[:5]]}
    data['counts']={k:int(v) for k,v in re.findall(r'(iterations|dropped_copperlists|persisted_copperlists|dropped_keyframes|persisted_keyframes)=(\d+)',(root/(key+'.txt')).read_text())}
    data['counts']['offered']=data['counts']['iterations']
    data['counts']['accepted']=data['counts']['offered']-data['counts']['dropped_copperlists']
    if types['store']:
        # Snapshot dirty pages at syscall entry, retire only the generation observed then.
        dirty={};claimed={};peak=0
        events=sorted([(e['end_ns'],0,i,e) for i,e in enumerate(types['store'])]+[(f[k],t,i,f) for i,f in enumerate(flush) for k,t in [('begin_ns',1),('end_ns',2)]])
        for at,t,i,e in events:
            keys=[(e['slab'],p) for p in range(e['offset']//page,(e['offset']+e['bytes']-1)//page+1)]
            if t==0:
                for k in keys:dirty[k]=at
                peak=max(peak,len(dirty))
            elif t==1:claimed[i]=[(k,dirty[k]) for k in keys if k in dirty]
            else:
                for k,old in claimed.pop(i):
                    if dirty.get(k)==old:del dirty[k]
        data['peak_dirty_including_inflight_bytes']=peak*page
    all_runs[key]=data
(root/'metrics.json').write_text(json.dumps(all_runs,indent=2)+'\n')
groups=collections.defaultdict(list)
for k,v in all_runs.items():
    if not k.endswith('-spatial'):groups[k.rsplit('-',1)[0]].append(v)
with (root/'spread.csv').open('w') as f:
    w=csv.writer(f)
    w.writerow(['configuration','repeats','metric','min','max'])
    for key,group in groups.items():
        for label,fn in [('tick p99 ns',lambda r:r['times_ns']['tick']['p99']),('max tick ns',lambda r:r['times_ns']['tick']['max']),('missed ticks',lambda r:r['over_50ms']),('drops',lambda r:r['counts']['dropped_copperlists']),('final drain ns',lambda r:r['times_ns']['final_drain']['max']),('total requested bytes',lambda r:r['msync']['total_bytes'])]:
            vals=[fn(r) for r in group]
            w.writerow([key,len(group),label,min(vals),max(vals)])
