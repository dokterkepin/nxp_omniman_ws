import React from 'react';
import clsx from 'clsx';
import { useTheme } from '../contexts/ThemeContext';

export default function ThemeToggle() {
  const { theme, setTheme } = useTheme();

  const buttonClass = (value) =>
    clsx(
      'flex-1',
      'border-0',
      'px-1',
      'py-1.5',
      'text-[10px]',
      'font-semibold',
      'leading-none',
      'transition-colors',
      'cursor-pointer',
      {
        'bg-white text-gray-900 shadow-sm dark:bg-slate-700 dark:text-white': theme === value,
        'bg-transparent text-gray-500 hover:text-gray-900 dark:text-slate-400 dark:hover:text-white':
          theme !== value,
      }
    );

  return (
    <div
      className="flex w-full overflow-hidden rounded-lg border border-gray-200 bg-gray-200/80 p-0.5 shadow-inner dark:border-slate-700 dark:bg-slate-900"
      role="group"
      aria-label="Color theme"
    >
      <button
        type="button"
        className={buttonClass('light')}
        onClick={() => setTheme('light')}
        aria-pressed={theme === 'light'}
        title="Light theme"
      >
        Light
      </button>
      <button
        type="button"
        className={buttonClass('dark')}
        onClick={() => setTheme('dark')}
        aria-pressed={theme === 'dark'}
        title="Dark theme"
      >
        Dark
      </button>
    </div>
  );
}
