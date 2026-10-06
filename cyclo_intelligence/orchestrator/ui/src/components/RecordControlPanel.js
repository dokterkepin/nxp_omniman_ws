// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");

import React, { useEffect, useState } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import { RecordPhase } from '../constants/taskPhases';

const phaseGuideMessages = {
  [RecordPhase.READY]: 'Ready',
  [RecordPhase.RECORDING]: 'Recording',
  [RecordPhase.SAVING]: 'Saving...',
  [RecordPhase.PAUSED]: 'Paused',
};

const spinnerFrames = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧'];

export default function RecordControlPanel() {
  const recordStatus = useSelector((state) => state.tasks.recordStatus);
  const [spinnerIndex, setSpinnerIndex] = useState(0);

  const phase = recordStatus.recordPhase;
  const isRecording = phase === RecordPhase.RECORDING;
  const isSaving = phase === RecordPhase.SAVING;
  const isBusy = isRecording || isSaving;
  const subtaskCount = Number(recordStatus.subtaskCount || 0);
  const currentSubtaskIndex = Number(recordStatus.currentSubtaskIndex || 0);
  const hasSubtasks = subtaskCount > 0;

  useEffect(() => {
    setSpinnerIndex((prev) => (prev + 1) % spinnerFrames.length);
  }, [recordStatus]);

  const classBody = clsx(
    'bg-white/90',
    'backdrop-blur-sm',
    'rounded-full',
    'px-3',
    'py-1',
    'flex',
    'flex-row',
    'items-center',
    'gap-1.5',
    'shadow-md',
    'border',
    'border-gray-100'
  );

  return (
    <div className={classBody}>
      <span className="text-lg font-semibold text-gray-500 whitespace-nowrap px-1 shrink-0">
        Record
      </span>
      <div className="w-px h-2/3 bg-gray-300 shrink-0"></div>

      <div className="flex items-center gap-1 shrink-0 px-1">
        <span className="text-gray-600 font-semibold text-lg whitespace-nowrap">
          {phaseGuideMessages[phase] || ''}
        </span>
        {isBusy && (
          <span className="font-mono text-blue-500 text-sm">
            {spinnerFrames[spinnerIndex]}
          </span>
        )}
      </div>

      {isRecording && (
        <>
          <div className="w-px h-2/3 bg-gray-400 shrink-0"></div>
          <div className="flex items-center gap-1 shrink-0 px-1">
            <span className="text-gray-500 text-lg font-medium">
              {recordStatus.proceedTime}s
            </span>
          </div>
        </>
      )}

      <div className="w-px h-2/3 bg-gray-400 shrink-0"></div>
      <div className="flex items-center gap-1 shrink-0 px-1">
        <span className="text-gray-500 text-lg font-medium">EP</span>
        <span className="bg-gray-100 rounded px-1.5 py-0.5 text-lg font-bold">
          {recordStatus.currentEpisodeNumber}
        </span>
      </div>

      {hasSubtasks && (
        <>
          <div className="w-px h-2/3 bg-gray-400 shrink-0"></div>
          <div className="flex items-center gap-1 shrink-0 px-1 max-w-[260px]">
            <span className="text-gray-500 text-lg font-medium whitespace-nowrap">
              ST {Math.min(currentSubtaskIndex + 1, subtaskCount)}/{subtaskCount}
            </span>
            {recordStatus.currentSubtaskInstruction && (
              <span className="text-sm text-gray-600 truncate">
                {recordStatus.currentSubtaskInstruction}
              </span>
            )}
          </div>
        </>
      )}
    </div>
  );
}
