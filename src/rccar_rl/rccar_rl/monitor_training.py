#!/usr/bin/env python3
"""
monitor_training.py
===================
RL 학습 모니터링 도구
TensorBoard 자동 시작 + 학습 통계 웹 UI
"""

import os
import sys
import json
import time
import argparse
import subprocess
import threading
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler
from datetime import datetime

# ──────────────────────────────────────────────────────────────
# 설정
# ──────────────────────────────────────────────────────────────
BASE_DIR = os.path.expanduser('~/rl_training')
LOG_DIR = os.path.join(BASE_DIR, 'logs')
MODEL_DIR = os.path.join(BASE_DIR, 'models')
BEST_DIR = os.path.join(BASE_DIR, 'best_model')
WEB_PORT = 8765


# ──────────────────────────────────────────────────────────────
# 학습 통계 수집
# ──────────────────────────────────────────────────────────────
class TrainingStatsCollector:
    """TensorBoard 로그에서 학습 통계 수집"""

    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.monitor_file = self._find_monitor_file(log_dir)

    def _find_monitor_file(self, log_dir):
        """가장 최근에 수정된 .monitor.csv 파일 탐색."""
        if not os.path.exists(log_dir):
            return None
        candidates = sorted(
            [os.path.join(log_dir, f) for f in os.listdir(log_dir)
             if f.endswith('.monitor.csv')],
            key=os.path.getmtime,
        )
        return candidates[-1] if candidates else None

    def get_latest_stats(self):
        """최신 학습 통계 반환"""
        stats = {
            'timestamp': datetime.now().isoformat(),
            'models': self._get_model_info(),
            'monitor': self._get_monitor_stats(),
            'tensorboard_log_dir': self.log_dir,
        }
        return stats

    def _get_model_info(self):
        """저장된 모델 정보"""
        models = []
        if os.path.exists(MODEL_DIR):
            for f in sorted(os.listdir(MODEL_DIR)):
                if f.endswith('.zip'):
                    path = os.path.join(MODEL_DIR, f)
                    size = os.path.getsize(path) / (1024 * 1024)  # MB
                    mtime = os.path.getmtime(path)
                    models.append({
                        'name': f,
                        'size_mb': round(size, 2),
                        'modified': datetime.fromtimestamp(mtime).isoformat(),
                    })
        return models[-5:] if models else []  # 최신 5개만

    def _get_monitor_stats(self):
        """Monitor CSV 파일에서 에피소드 통계"""
        if not self.monitor_file or not os.path.exists(self.monitor_file):
            return None

        try:
            lines = []
            with open(self.monitor_file, 'r') as f:
                lines = f.readlines()

            if len(lines) < 3:
                return None

            # 마지막 에피소드들 파싱
            episodes = []
            for line in lines[2:]:  # 헤더 2줄 스킵
                parts = line.strip().split(',')
                if len(parts) >= 3:
                    episodes.append({
                        'timestep': int(float(parts[2])),
                        'episode_reward': float(parts[0]),
                        'episode_length': int(float(parts[1])),
                    })

            if not episodes:
                return None

            # 통계 계산
            recent_episodes = episodes[-10:] if len(episodes) > 10 else episodes
            avg_reward = sum(e['episode_reward'] for e in recent_episodes) / len(recent_episodes)
            avg_length = sum(e['episode_length'] for e in recent_episodes) / len(recent_episodes)

            return {
                'total_episodes': len(episodes),
                'last_episode': {
                    'reward': recent_episodes[-1]['episode_reward'],
                    'length': recent_episodes[-1]['episode_length'],
                    'timestep': recent_episodes[-1]['timestep'],
                },
                'recent_avg_reward': round(avg_reward, 2),
                'recent_avg_length': round(avg_length, 1),
            }
        except Exception as e:
            print(f'[monitor] 모니터 파일 파싱 오류: {e}')
            return None


# ──────────────────────────────────────────────────────────────
# 웹 UI 서버
# ──────────────────────────────────────────────────────────────
class MonitoringHandler(SimpleHTTPRequestHandler):
    """학습 통계를 JSON으로 제공하는 HTTP 핸들러"""

    stats_collector = None

    def do_GET(self):
        if self.path == '/api/stats':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            stats = self.stats_collector.get_latest_stats()
            self.wfile.write(json.dumps(stats, indent=2).encode())
        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(get_html_dashboard().encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # 너무 많은 로그 생략
        pass


# ──────────────────────────────────────────────────────────────
# HTML 대시보드
# ──────────────────────────────────────────────────────────────
def get_html_dashboard():
    """RL 모니터링 웹 대시보드"""
    return '''<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RC-Car RL 모니터링</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            color: white;
            margin-bottom: 30px;
        }
        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
        }
        .header p {
            font-size: 1.1em;
            opacity: 0.9;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        .card {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
            transition: transform 0.3s ease;
        }
        .card:hover {
            transform: translateY(-5px);
        }
        .card h3 {
            color: #667eea;
            margin-bottom: 15px;
            font-size: 1.2em;
        }
        .stat {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 0;
            border-bottom: 1px solid #eee;
        }
        .stat:last-child {
            border-bottom: none;
        }
        .stat-label {
            color: #666;
            font-weight: 600;
        }
        .stat-value {
            color: #333;
            font-size: 1.1em;
            font-weight: bold;
        }
        .stat-unit {
            color: #999;
            font-size: 0.9em;
            margin-left: 5px;
        }
        .model-list {
            max-height: 300px;
            overflow-y: auto;
        }
        .model-item {
            padding: 10px;
            margin: 5px 0;
            background: #f5f5f5;
            border-left: 4px solid #667eea;
            border-radius: 4px;
            font-size: 0.9em;
        }
        .model-name {
            font-weight: bold;
            color: #333;
        }
        .model-info {
            color: #666;
            font-size: 0.85em;
            margin-top: 5px;
        }
        .links {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 30px;
        }
        .btn {
            display: inline-block;
            padding: 12px 20px;
            background: white;
            color: #667eea;
            text-decoration: none;
            border-radius: 8px;
            font-weight: bold;
            text-align: center;
            transition: all 0.3s ease;
            border: 2px solid #667eea;
        }
        .btn:hover {
            background: #667eea;
            color: white;
        }
        .status {
            padding: 5px 10px;
            border-radius: 4px;
            font-size: 0.9em;
            font-weight: bold;
        }
        .status.active {
            background: #d4edda;
            color: #155724;
        }
        .status.inactive {
            background: #f8d7da;
            color: #721c24;
        }
        .loading {
            text-align: center;
            color: #999;
            padding: 20px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 RC-Car 강화학습 모니터링</h1>
            <p>AWS RoboMaker Bookstore Map · PPO · Gymnasium</p>
        </div>

        <div id="content" class="loading">
            데이터 로드 중...
        </div>

        <div class="links">
            <a href="http://localhost:6006" target="_blank" class="btn">📊 TensorBoard 열기</a>
            <a href="#" class="btn" onclick="location.reload()">🔄 새로고침</a>
        </div>
    </div>

    <script>
        async function updateStats() {
            try {
                const response = await fetch('/api/stats');
                const stats = await response.json();
                renderDashboard(stats);
            } catch (error) {
                document.getElementById('content').innerHTML = 
                    '<p style="color: red;">데이터를 불러올 수 없습니다.</p>';
            }
        }

        function renderDashboard(stats) {
            let html = '<div class="grid">';

            // 최신 에피소드 통계
            if (stats.monitor) {
                html += `
                    <div class="card">
                        <h3>📈 최신 에피소드</h3>
                        <div class="stat">
                            <span class="stat-label">보상</span>
                            <span class="stat-value">${stats.monitor.last_episode.reward.toFixed(1)}</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">길이</span>
                            <span class="stat-value">${stats.monitor.last_episode.length}</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">타임스텝</span>
                            <span class="stat-value">${(stats.monitor.last_episode.timestep / 1000).toFixed(0)}<span class="stat-unit">K</span></span>
                        </div>
                    </div>
                `;
            }

            // 최근 평균
            if (stats.monitor) {
                html += `
                    <div class="card">
                        <h3>📊 최근 10 에피소드 평균</h3>
                        <div class="stat">
                            <span class="stat-label">평균 보상</span>
                            <span class="stat-value">${stats.monitor.recent_avg_reward.toFixed(1)}</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">평균 길이</span>
                            <span class="stat-value">${stats.monitor.recent_avg_length.toFixed(0)}</span>
                        </div>
                        <div class="stat">
                            <span class="stat-label">총 에피소드</span>
                            <span class="stat-value">${stats.monitor.total_episodes}</span>
                        </div>
                    </div>
                `;
            }

            // 모델 정보
            if (stats.models && stats.models.length > 0) {
                html += '<div class="card"><h3>💾 최신 모델</h3><div class="model-list">';
                stats.models.forEach(model => {
                    html += `
                        <div class="model-item">
                            <div class="model-name">${model.name}</div>
                            <div class="model-info">${model.size_mb} MB · ${new Date(model.modified).toLocaleString()}</div>
                        </div>
                    `;
                });
                html += '</div></div>';
            }

            // 상태
            html += `
                <div class="card">
                    <h3>⚙️ 시스템</h3>
                    <div class="stat">
                        <span class="stat-label">로그 디렉토리</span>
                        <span class="stat-value" style="font-size: 0.9em; text-align: right; word-break: break-all;">${stats.tensorboard_log_dir}</span>
                    </div>
                    <div class="stat">
                        <span class="stat-label">마지막 업데이트</span>
                        <span class="stat-value">${new Date(stats.timestamp).toLocaleTimeString()}</span>
                    </div>
                </div>
            `;

            html += '</div>';
            document.getElementById('content').innerHTML = html;
        }

        // 1초마다 업데이트
        updateStats();
        setInterval(updateStats, 1000);
    </script>
</body>
</html>
'''


# ──────────────────────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='RL 학습 모니터링 서버')
    parser.add_argument('--port', type=int, default=WEB_PORT,
                        help=f'웹 대시보드 포트 (기본: {WEB_PORT})')
    parser.add_argument('--tensorboard-port', type=int, default=6006,
                        help='TensorBoard 포트 (기본: 6006)')
    parser.add_argument('--no-tensorboard', action='store_true',
                        help='TensorBoard 자동 시작 안 함')
    args = parser.parse_args()

    # TensorBoard 시작
    if not args.no_tensorboard:
        print(f'[monitor] TensorBoard 시작 중... (포트 {args.tensorboard_port})')
        try:
            subprocess.Popen([
                'tensorboard',
                f'--logdir={LOG_DIR}',
                f'--port={args.tensorboard_port}',
                '--bind_all',
            ])
            print(f'[monitor] ✓ TensorBoard: http://localhost:{args.tensorboard_port}')
            time.sleep(2)
        except Exception as e:
            print(f'[monitor] ✗ TensorBoard 실행 오류: {e}')

    # 웹 서버 시작
    print(f'[monitor] 웹 대시보드 시작 중... (포트 {args.port})')
    MonitoringHandler.stats_collector = TrainingStatsCollector(LOG_DIR)

    server = HTTPServer(('0.0.0.0', args.port), MonitoringHandler)
    print(f'[monitor] ✓ 웹 대시보드: http://localhost:{args.port}')
    print(f'[monitor] 모니터링 시작. 종료: Ctrl+C')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[monitor] 종료 중...')
        server.shutdown()


if __name__ == '__main__':
    main()
