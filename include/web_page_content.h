#ifndef WEB_PAGE_CONTENT_H
#define WEB_PAGE_CONTENT_H

const char *HTML_CONTENT = R"rawliteral(
    <!DOCTYPE html>
    <html lang="zh-Hant">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>ESP32 Drone PID Tuning</title>
        <style>
            body { font-family: sans-serif; padding: 15px; background-color: #f4f4f4; }
            .container { max-width: 800px; margin: auto; background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
            h1, h2 { text-align: center; color: #333; }
            .pid-group { border: 1px solid #ddd; border-radius: 5px; margin-bottom: 20px; padding: 15px; background-color: #fafafa; }
            .pid-group h3 { margin-top: 0; color: #555; border-bottom: 1px solid #eee; padding-bottom: 5px; }
            label { display: inline-block; width: 40px; font-weight: bold; color: #666; }
            input[type=number] { width: 80px; padding: 8px; margin: 5px 10px 5px 0; border: 1px solid #ccc; border-radius: 4px; }
            .button-group { text-align: center; margin-top: 20px; }
            button { padding: 10px 20px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; font-size: 1em; }
            .btn-load { background-color: #4CAF50; color: white; }
            .btn-apply { background-color: #007bff; color: white; }
            .btn-save { background-color: #ffc107; color: #333; }
            .status { margin-top: 15px; text-align: center; font-weight: bold; min-height: 1.2em; }
            .status.success { color: green; }
            .status.error { color: red; }
            .disabled { opacity: 0.6; cursor: not-allowed; }
            .loader { border: 4px solid #f3f3f3; border-top: 4px solid #3498db; border-radius: 50%; width: 20px; height: 20px; animation: spin 1s linear infinite; display: inline-block; vertical-align: middle; margin-left: 10px; visibility: hidden; }
            @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>ESP32 無人機 PID 調參</h1>
            <div class="status" id="status-message"></div>
            <div class="loader" id="loader"></div>
    
            <div class="button-group">
                <button class="btn-load" onclick="loadPIDs()">讀取目前參數</button>
                <button class="btn-save" onclick="savePIDs()">儲存到飛控</button>
                <p style="font-size: 0.8em; color: #888;">(注意：儲存和套用僅在 Disarmed 狀態下有效)</p>
            </div>
    
            <div id="pid-forms">
                </div>
    
        </div>
    
        <script>
            const pidLoops = [
                { id: 0, name: "Rate Roll" }, { id: 1, name: "Rate Pitch" }, { id: 2, name: "Rate Yaw" },
                { id: 3, name: "Angle Roll" }, { id: 4, name: "Angle Pitch" }, { id: 5, name: "Angle Yaw" }
            ];
            const statusElement = document.getElementById('status-message');
            const loaderElement = document.getElementById('loader');
            const pidFormsContainer = document.getElementById('pid-forms');
    
            function showLoader(show) {
                loaderElement.style.visibility = show ? 'visible' : 'hidden';
            }
    
            function setStatus(message, isError = false) {
                statusElement.textContent = message;
                statusElement.className = 'status ' + (isError ? 'error' : 'success');
                setTimeout(() => statusElement.textContent = '', 5000); // 5秒後清除訊息
            }
    
            // 動態生成 PID 表單
            function generateForms() {
                let html = '';
                pidLoops.forEach(loop => {
                    html += `
                    <div class="pid-group">
                        <h3>${loop.name} (ID: ${loop.id})</h3>
                        <form id="form-${loop.id}" onsubmit="event.preventDefault(); applySinglePID(${loop.id});">
                            <label for="kp-${loop.id}">Kp:</label><input type="number" step="0.01" id="kp-${loop.id}" name="kp" required>
                            <label for="ki-${loop.id}">Ki:</label><input type="number" step="0.01" id="ki-${loop.id}" name="ki" required>
                            <label for="kd-${loop.id}">Kd:</label><input type="number" step="0.01" id="kd-${loop.id}" name="kd" required>
                            <button type="submit" class="btn-apply">套用 ${loop.name}</button>
                        </form>
                        <div style="font-size:0.8em; color: #777; margin-top: 5px;">
                            Integral Limit: <span id="intlimit-${loop.id}">N/A</span>, Output Limit: <span id="outlimit-${loop.id}">N/A</span>
                        </div>
                    </div>`;
                });
                pidFormsContainer.innerHTML = html;
            }
    
            async function loadPIDs() {
                showLoader(true);
                setStatus('正在讀取參數...');
                try {
                    const response = await fetch('/api/pids');
                    if (!response.ok) {
                        throw new Error(`HTTP 錯誤! 狀態: ${response.status}`);
                    }
                    const data = await response.json();
                    if (data.error) {
                         throw new Error(data.error);
                    }
                    // 填充表單
                    pidLoops.forEach(loop => {
                        const pidData = data.pids[loop.id];
                        if (pidData) {
                            document.getElementById(`kp-${loop.id}`).value = pidData.kp.toFixed(3);
                            document.getElementById(`ki-${loop.id}`).value = pidData.ki.toFixed(3);
                            document.getElementById(`kd-${loop.id}`).value = pidData.kd.toFixed(3);
                            document.getElementById(`intlimit-${loop.id}`).textContent = pidData.int_limit.toFixed(1);
                            document.getElementById(`outlimit-${loop.id}`).textContent = pidData.out_limit.toFixed(1);
                        }
                    });
                    setStatus('參數讀取成功！');
                } catch (error) {
                    console.error('讀取 PID 失敗:', error);
                    setStatus(`讀取失敗: ${error.message}`, true);
                } finally {
                    showLoader(false);
                }
            }
    
            async function applySinglePID(loopId) {
                showLoader(true);
                setStatus('正在套用參數...');
                const form = document.getElementById(`form-${loopId}`);
                const kp = parseFloat(form.kp.value);
                const ki = parseFloat(form.ki.value);
                const kd = parseFloat(form.kd.value);
    
                if (isNaN(kp) || isNaN(ki) || isNaN(kd)) {
                    setStatus('無效的數值輸入！', true);
                    showLoader(false);
                    return;
                }
    
                const payload = {
                    loop: loopId,
                    kp: kp,
                    ki: ki,
                    kd: kd
                };
    
                try {
                    const response = await fetch('/api/pids', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify(payload)
                    });
                    const result = await response.json();
                    if (!response.ok) {
                        throw new Error(result.error || `HTTP 錯誤! 狀態: ${response.status}`);
                    }
                    setStatus(`PID ${loopId} 套用成功！`);
                } catch (error) {
                    console.error('套用 PID 失敗:', error);
                    setStatus(`套用失敗: ${error.message}`, true);
                } finally {
                    showLoader(false);
                }
            }
    
            async function savePIDs() {
                showLoader(true);
                setStatus('正在儲存參數到 NVS...');
                try {
                    const response = await fetch('/api/pids/save', { method: 'POST' });
                    const result = await response.json();
                    if (!response.ok) {
                         throw new Error(result.error || `HTTP 錯誤! 狀態: ${response.status}`);
                    }
                    setStatus('參數成功儲存到飛控 NVS！');
                } catch (error) {
                    console.error('儲存 PID 失敗:', error);
                    setStatus(`儲存失敗: ${error.message}`, true);
                } finally {
                    showLoader(false); 
                }
            }
    
            // 頁面載入時生成表單並嘗試讀取一次參數
            window.onload = () => {
                generateForms();
                loadPIDs();
            };
        </script>
    </body>
    </html>
    )rawliteral"; // 結束原始字串

#endif // WEB_PAGE_CONTENT_H