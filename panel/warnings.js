// System warnings popup — fetches /api/warnings and shows critical alerts
(function(){
    var shown = {};
    function checkWarnings(){
        fetch('/api/warnings').then(function(r){return r.json()}).then(function(d){
            if(!d.warnings||!d.warnings.length) return;
            d.warnings.forEach(function(w){
                if(shown[w.code]) return;
                shown[w.code]=true;
                var overlay=document.createElement('div');
                overlay.style.cssText='position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);z-index:99999;display:flex;align-items:center;justify-content:center';
                var box=document.createElement('div');
                box.style.cssText='background:#1a0000;border:3px solid #ff4444;border-radius:12px;padding:24px 32px;max-width:600px;width:90%;color:#fff;font-family:sans-serif;box-shadow:0 0 40px rgba(255,0,0,0.3)';
                box.innerHTML='<div style="display:flex;align-items:center;gap:12px;margin-bottom:16px">'
                    +'<span style="font-size:36px">⚠️</span>'
                    +'<h2 style="margin:0;color:#ff4444;font-size:20px">'+w.title+'</h2>'
                    +'</div>'
                    +'<p style="color:#ffcccc;font-size:14px;line-height:1.6;margin:0 0 20px">'+w.message+'</p>'
                    +'<button onclick="this.parentNode.parentNode.remove()" style="background:#ff4444;color:#fff;border:none;padding:8px 24px;border-radius:6px;cursor:pointer;font-size:14px;font-weight:600">OK</button>';
                overlay.appendChild(box);
                document.body.appendChild(overlay);
            });
        }).catch(function(){});
    }
    if(document.readyState==='loading'){document.addEventListener('DOMContentLoaded',checkWarnings)}
    else{checkWarnings()}
})();
